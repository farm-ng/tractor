#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"

#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/image.pb.h"
#include "farm_ng_proto/tractor/v1/tracking_camera.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::CaptureVideoDatasetConfiguration;
using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::CaptureVideoDatasetStatus;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::Subscription;
using farm_ng_proto::tractor::v1::TrackingCameraCommand;

void Cleanup(farm_ng::EventBus& bus) {
  farm_ng::RequestStopCapturing(bus);
  LOG(INFO) << "Requested Stop capture";
  farm_ng::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}

namespace farm_ng {
class CaptureVideoDatasetProgram {
 public:
  CaptureVideoDatasetProgram(EventBus& bus,
                             CaptureVideoDatasetConfiguration configuration,
                             bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions(
        {bus_.GetName(), "tracking_camera/front/left/image", "logger/status"});

    bus_.GetEventSignal()->connect(std::bind(
        &CaptureVideoDatasetProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {"ipc_logger", "tracking_camera"});
    LoggingStatus log = StartLogging(bus_, configuration_.name());
    RequestStartCapturing(bus_,
                          TrackingCameraCommand::RecordStart::MODE_EVERY_FRAME);

    try {
      bus_.get_io_service().run();
    } catch (std::exception& e) {
      CaptureVideoDatasetResult result;
      result.mutable_configuration()->CopyFrom(configuration_);
      result.set_num_frames(status_.num_frames());
      result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
      result.mutable_dataset()->set_path(log.recording().archive_path());

      // TODO some how save the result in the archive directory as well, so its
      // self contained.
      ArchiveProtobufAsJsonResource(configuration_.name(), result);

      status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
          BucketId::kVideoDatasets, configuration_.name(), result));
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

  bool on_image(const EventPb& event) {
    Image image;
    if (!event.data().UnpackTo(&image)) {
      return false;
    }

    status_.set_num_frames(status_.num_frames() + 1);

    return true;
  }

  bool on_configuration(const EventPb& event) {
    CaptureVideoDatasetConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CaptureVideoDatasetConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    if (event.name().rfind("tracking_camera/front/left/image", 0) == 0) {
      if (on_image(event)) {
        return;
      }
    }
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CaptureVideoDatasetConfiguration configuration_;
  CaptureVideoDatasetStatus status_;
};

}  // namespace farm_ng

int Main(farm_ng::EventBus& bus) {
  CaptureVideoDatasetConfiguration config;
  config.set_name(FLAGS_name);
  farm_ng::CaptureVideoDatasetProgram program(bus, config, FLAGS_interactive);
  return program.run();
}

int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
