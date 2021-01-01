// A demonstration "Countdown" program

#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

DEFINE_bool(interactive, false, "Receive program args via eventbus");
DEFINE_string(name, "default", "A name for the result file");
DEFINE_uint64(start, 10, "The starting number for the countdown");

namespace farm_ng {
namespace core {

class CountdownProgram {
 public:
  CountdownProgram(EventBus& bus, CountdownConfiguration configuration,
                   bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName(), "logger/command", "logger/status"});
    bus_.GetEventSignal()->connect(
        std::bind(&CountdownProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {"ipc_logger"});

    LoggingStatus log = StartLogging(bus_, configuration_.name());
    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    try {
      bus_.get_io_service().run();
    } catch (std::exception& e) {
      result.mutable_configuration()->CopyFrom(configuration_);
      result.set_num_frames(status_.num_frames());
      result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
      result.mutable_dataset()->set_path(log.recording().archive_path());

      // TODO some how save the result in the archive directory as well, so
      // its self contained.
      ArchiveProtobufAsJsonResource(configuration_.name(), result);

      status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
          BUCKET_VIDEO_DATASETS, configuration_.name(), result));
      LOG(INFO) << "Complete:\n" << status_.DebugString();
      send_status();
      return 0;
    }
    return 1;
  }

  void send_status() {
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&CaptureVideoDatasetProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    CountdownConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CountdownConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CountdownConfiguration configuration_;
  CountdownStatus status_;
  uint32_t counter_;
};

}  // namespace core
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::core::CountdownConfiguration config;
  config.set_name(FLAGS_name);
  farm_ng::core::CountdownProgram program(bus, config, FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) {
  farm_ng::core::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
