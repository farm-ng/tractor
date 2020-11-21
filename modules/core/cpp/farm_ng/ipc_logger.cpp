#include "farm_ng/core/event_log.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include <glog/logging.h>
#include <iostream>

typedef farm_ng::core::Event EventPb;

namespace farm_ng {
namespace core {

class IpcLogger {
 public:
  IpcLogger(EventBus& bus)
      : bus_(bus),
        log_writer_(nullptr),
        log_timer_(bus_.get_io_service()),
        announce_timer_(bus_.get_io_service()) {
    bus_.AddSubscriptions({".*"});
    bus_.GetEventSignal()->connect(
        std::bind(&IpcLogger::on_event, this, std::placeholders::_1));
    log_state(boost::system::error_code());
    log_announce(boost::system::error_code());
    logging_status_.mutable_stopped();
  }

  void on_command(const LoggingCommand& command) {
    last_command_.CopyFrom(command);
    switch (last_command_.command_case()) {
      case LoggingCommand::kRecordStart: {
        auto resource_path = GetUniqueArchiveResource("events", "log", "");
        log_resource_ = resource_path.first;
        LOG(INFO) << "Starting log: " << log_resource_.ShortDebugString();
        log_writer_.reset(new EventLogWriter(resource_path.second));
        auto recording = logging_status_.mutable_recording();
        recording->set_archive_path(resource_path.first.path());
        recording->set_path(resource_path.second.string());
        recording->set_n_messages(0);
        recording->mutable_stamp_begin()->CopyFrom(MakeTimestampNow());
        LOG(INFO) << "Status: " << logging_status_.ShortDebugString();

      } break;

      case LoggingCommand::kRecordStop:
        LOG(INFO) << "Stopping log: " << log_resource_.ShortDebugString() << " "
                  << logging_status_.ShortDebugString();
        log_writer_.reset();
        logging_status_.mutable_stopped();
        LOG(INFO) << "Status: " << logging_status_.ShortDebugString();
        break;

      case LoggingCommand::COMMAND_NOT_SET:
      default:
        break;
    }
    bus_.Send(MakeEvent("logger/status", logging_status_));
  }

  void on_event(const EventPb& event) {
    if (event.data().type_url() ==
        "type.googleapis.com/" + LoggingCommand::descriptor()->full_name()) {
      LoggingCommand command;
      CHECK(event.data().UnpackTo(&command));
      on_command(command);
    }

    switch (last_command_.command_case()) {
      case LoggingCommand::kRecordStart: {
        log_writer_->Write(event);
        auto recording = logging_status_.mutable_recording();
        recording->set_n_messages(recording->n_messages() + 1);
        VLOG(1) << "Status: " << logging_status_.ShortDebugString();
      } break;

      case LoggingCommand::kRecordStop:
        break;

      case LoggingCommand::COMMAND_NOT_SET:
      default:
        break;
    }
  }
  void log_announce(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "announce timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    announce_timer_.expires_from_now(boost::posix_time::seconds(10));
    announce_timer_.async_wait(
        std::bind(&IpcLogger::log_announce, this, std::placeholders::_1));

    for (const auto& it : bus_.GetAnnouncements()) {
      VLOG(1) << it.second.ShortDebugString();
    }
  }

  void log_state(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "log timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    log_timer_.expires_from_now(boost::posix_time::seconds(1));
    log_timer_.async_wait(
        std::bind(&IpcLogger::log_state, this, std::placeholders::_1));
    bus_.Send(MakeEvent("logger/status", logging_status_));
    VLOG(1) << logging_status_.ShortDebugString();
    for (const auto& it : bus_.GetState()) {
      VLOG(1) << it.first << " : " << it.second.ShortDebugString();
    }
  }

 private:
  EventBus& bus_;
  std::unique_ptr<EventLogWriter> log_writer_;

  boost::asio::deadline_timer log_timer_;
  boost::asio::deadline_timer announce_timer_;
  Resource log_resource_;
  LoggingCommand last_command_;
  LoggingStatus logging_status_;
};

}  // namespace core
}  // namespace farm_ng

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::core::IpcLogger logger(bus);
  bus.get_io_service().run();
  return 0;
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
