#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/capture_calibration_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/tracking_camera.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");
DEFINE_int32(num_frames, 16, "number of frames to capture");

typedef farm_ng_proto::tractor::v1::Event EventPb;

using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::CaptureCalibrationDatasetConfiguration;
using farm_ng_proto::tractor::v1::CaptureCalibrationDatasetResult;
using farm_ng_proto::tractor::v1::CaptureCalibrationDatasetStatus;
using farm_ng_proto::tractor::v1::Subscription;
using farm_ng_proto::tractor::v1::TrackingCameraCommand;

namespace farm_ng {
class CaptureCalibrationDatasetProgram {
 public:
  CaptureCalibrationDatasetProgram(
      EventBus& bus, CaptureCalibrationDatasetConfiguration configuration,
      bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({"^calibrator/"});
    bus_.GetEventSignal()->connect(
        std::bind(&CaptureCalibrationDatasetProgram::on_event, this,
                  std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {"ipc_logger", "tracking_camera"});
    LoggingStatus log = StartLogging(bus_, configuration_.name());
    RequestStartCapturing(
        bus_, TrackingCameraCommand::RecordStart::MODE_APRILTAG_STABLE);

    while (status_.num_frames() < configuration_.num_frames()) {
      bus_.get_io_service().run_one();
    }

    CaptureCalibrationDatasetResult result;
    result.mutable_configuration()->CopyFrom(configuration_);
    result.set_num_frames(status_.num_frames());
    result.mutable_tag_ids()->CopyFrom(status_.tag_ids());
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    result.mutable_dataset()->set_path(log.recording().archive_path());

    // TODO some how save the result in the archive directory as well, so its
    // self contained.
    ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        BucketId::kCalibrationDatasets, configuration_.name(), result));
    LOG(INFO) << "Complete:\n" << status_.DebugString();
    send_status();
    return 0;
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
    timer_.async_wait(std::bind(&CaptureCalibrationDatasetProgram::on_timer,
                                this, std::placeholders::_1));

    send_status();
  }

  bool on_apriltag_detections(const EventPb& event) {
    ApriltagDetections detections;
    if (!event.data().UnpackTo(&detections)) {
      return false;
    }

    if (detections.detections().size() == 0) {
      return true;
    }

    status_.set_num_frames(status_.num_frames() + 1);
    for (auto detection : detections.detections()) {
      int tag_id = detection.id();
      if (std::find(status_.tag_ids().begin(), status_.tag_ids().end(),
                    tag_id) == status_.tag_ids().end()) {
        status_.add_tag_ids(tag_id);
      }
    }

    send_status();
    return true;
  }

  bool on_configuration(const EventPb& event) {
    CaptureCalibrationDatasetConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CaptureCalibrationDatasetConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    // using 'calibrator' for compatibility w/ existing code
    if (event.name().rfind("calibrator/", 0) == 0) {
      if (on_apriltag_detections(event)) {
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
  CaptureCalibrationDatasetConfiguration configuration_;
  CaptureCalibrationDatasetStatus status_;
};

}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) {
  farm_ng::RequestStopCapturing(bus);
  LOG(INFO) << "Requested Stop capture";
  farm_ng::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}
int Main(farm_ng::EventBus& bus) {
  CaptureCalibrationDatasetConfiguration config;
  config.set_num_frames(FLAGS_num_frames);
  config.set_name(FLAGS_name);

  farm_ng::CaptureCalibrationDatasetProgram program(bus, config,
                                                    FLAGS_interactive);
  return program.run();
}

int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
