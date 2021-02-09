// # playback at 0.1 speed realtime
// log_playback --loop=false --speed=0.1 --log= /tmp/farm-ng-event.log
// # playback realtime
// log_playback --loop=false --speed=1 --log= /tmp/farm-ng-event.log
// # playback realtime and loop
// log_playback --loop=true --speed=1 --log= /tmp/farm-ng-event.log
// # playback at 10x speed and publish on event bus, don't do this with robot
// not-estopped as this will send steering commands! log_playback --send
// --loop=true --speed=10 --log= /tmp/farm-ng-event.log

#include <gflags/gflags.h>
#include <google/protobuf/util/time_util.h>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/optional.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>

#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/core/log_playback.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");

DEFINE_string(
    log, "",
    "Path to log file, recorded with ipc_logger, relative to BLOBSTORE_ROOT");

DEFINE_bool(loop, false, "Loop?");

DEFINE_bool(send, false, "Send on event bus?");

DEFINE_double(speed, 1.0, "How fast to play log, multiple of realtime");

namespace farm_ng {
namespace core {

std::chrono::microseconds ToChronoDuration(
    google::protobuf::Duration duration) {
  return std::chrono::microseconds(
      google::protobuf::util::TimeUtil::DurationToMicroseconds(duration));
}

class IpcLogPlayback {
 public:
  IpcLogPlayback(EventBus& bus, const LogPlaybackConfiguration& configuration,
                 bool interactive)
      : io_service_(bus.get_io_service()),
        bus_(bus),
        status_timer_(bus.get_io_service()),
        log_timer_(bus.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName()});

    bus_.GetEventSignal()->connect(
        std::bind(&IpcLogPlayback::on_event, this, std::placeholders::_1));
    on_status_timer(boost::system::error_code());
  }

  void send_status() {
    status_.clear_message_stats();
    for (auto name_stats : message_stats_) {
      status_.add_message_stats()->CopyFrom(name_stats.second);
    }

    LOG(INFO) << status_.ShortDebugString();
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_status_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    status_timer_.expires_from_now(std::chrono::milliseconds(1000));
    status_timer_.async_wait(std::bind(&IpcLogPlayback::on_status_timer, this,
                                       std::placeholders::_1));

    send_status();
  }

  void on_event(const EventPb& event) {
    LogPlaybackConfiguration configuration;
    if (event.data().UnpackTo(&configuration)) {
      LOG(INFO) << configuration.ShortDebugString();
      set_configuration(configuration);
    }
  }

  void set_configuration(LogPlaybackConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    status_.mutable_configuration()->CopyFrom(configuration_);
    send_status();
  }

  void log_read_and_send(const boost::system::error_code& error) {
    if (error) {
      std::cerr << "log_timer_ error: " << __PRETTY_FUNCTION__ << error
                << std::endl;
      return;
    }
    if (next_message_) {
      *next_message_->mutable_stamp() = MakeTimestampNow();
      status_.set_message_count(status_.message_count() + 1);
      status_.mutable_last_message_stamp()->CopyFrom(next_message_->stamp());
      MessageStats& stats = message_stats_[next_message_->name()];
      stats.set_name(next_message_->name());
      stats.set_type_url(next_message_->data().type_url());
      stats.set_count(stats.count() + 1);
      double delta_seconds =
          google::protobuf::util::TimeUtil::DurationToNanoseconds(
              (next_message_->stamp() - stats.last_stamp())) *
          1.0e-9;
      if (delta_seconds < 1.0e-9 || delta_seconds > 60 * 60 * 60) {
        stats.set_frequency(0.0);
      } else {
        if (stats.frequency() < 1.0e-9) {
          stats.set_frequency(1.0 / delta_seconds);
        } else {
          // Take a windowed rolling average, exponential decay of older samples
          double alpha = 0.1;
          stats.set_frequency((1.0 / delta_seconds) * alpha +
                              stats.frequency() * (1 - alpha));
        }
      }
      stats.mutable_last_stamp()->CopyFrom(next_message_->stamp());

      if (configuration_.send()) {
        if (!next_message_->data().Is<LoggingCommand>()) {
          next_message_->set_name(std::string("playback/") +
                                  next_message_->name());
          bus_.Send(*next_message_);
        }
      }
    }
    try {
      next_message_ = log_reader_->ReadNext();
      LOG(INFO) << next_message_->name();
    } catch (std::runtime_error& e) {
      if (configuration_.loop()) {
        log_reader_.reset(new EventLogReader(configuration_.log()));
        next_message_.reset();
        io_service_.post(
            [this] { this->log_read_and_send(boost::system::error_code()); });
        return;
      } else {
        throw;
      }
    }
    if (!last_message_stamp_) {
      last_message_stamp_ = next_message_->stamp();
    }

    log_timer_.expires_from_now(std::chrono::microseconds(

        int64_t(ToChronoDuration(next_message_->stamp() - *last_message_stamp_)
                    .count() /
                configuration_.speed())));
    log_timer_.async_wait(std::bind(&IpcLogPlayback::log_read_and_send, this,
                                    std::placeholders::_1));
    last_message_stamp_ = next_message_->stamp();
  }

  void run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }
    log_reader_.reset(new EventLogReader(configuration_.log()));
    log_read_and_send(boost::system::error_code());

    bus_.get_io_service().run();
  }

 private:
  boost::asio::io_service& io_service_;
  EventBus& bus_;
  boost::asio::steady_timer status_timer_;
  boost::asio::steady_timer log_timer_;

  LogPlaybackConfiguration configuration_;
  LogPlaybackStatus status_;
  std::unique_ptr<EventLogReader> log_reader_;
  boost::optional<google::protobuf::Timestamp> last_message_stamp_;
  boost::optional<farm_ng::core::Event> next_message_;
  std::map<std::string, MessageStats> message_stats_;
};

}  // namespace core
}  // namespace farm_ng

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::core::LogPlaybackConfiguration configuration;
  configuration.set_loop(FLAGS_loop);
  configuration.mutable_log()->set_path(FLAGS_log);
  configuration.mutable_log()->set_content_type(
      "application/farm_ng.eventlog.v1");
  configuration.set_send(FLAGS_send);
  configuration.set_speed(FLAGS_speed);
  farm_ng::core::IpcLogPlayback playback(bus, configuration, FLAGS_interactive);
  playback.run();
  return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
