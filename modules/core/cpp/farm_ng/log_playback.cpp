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

DEFINE_string(log, "/tmp/farm-ng-event.log",
              "Path to log file, recorded with ipc_logger");

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
  IpcLogPlayback(EventBus& bus)
      : io_service_(bus.get_io_service()),
        bus_(bus),
        log_reader_(FLAGS_log),
        log_timer_(bus.get_io_service()) {
    log_read_and_send(boost::system::error_code());
  }

  void log_read_and_send(const boost::system::error_code& error) {
    if (error) {
      std::cerr << "log_timer_ error: " << __PRETTY_FUNCTION__ << error
                << std::endl;
      return;
    }
    if (next_message_) {
      *next_message_->mutable_stamp() = MakeTimestampNow();
      std::cout << next_message_->ShortDebugString() << std::endl;
      if (FLAGS_send) {
        bus_.Send(*next_message_);
      }
    }
    try {
      next_message_ = log_reader_.ReadNext();
    } catch (std::runtime_error& e) {
      if (FLAGS_loop) {
        log_reader_.Reset(FLAGS_log);
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
                FLAGS_speed)));
    log_timer_.async_wait(std::bind(&IpcLogPlayback::log_read_and_send, this,
                                    std::placeholders::_1));
    last_message_stamp_ = next_message_->stamp();
  }

 private:
  boost::asio::io_service& io_service_;
  EventBus& bus_;
  EventLogReader log_reader_;
  boost::asio::steady_timer log_timer_;
  boost::optional<google::protobuf::Timestamp> last_message_stamp_;
  boost::optional<farm_ng::core::Event> next_message_;
};

}  // namespace core
}  // namespace farm_ng

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::core::IpcLogPlayback playback(bus);
  bus.get_io_service().run();
  return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
