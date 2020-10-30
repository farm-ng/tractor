#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/image.pb.h"
#include "farm_ng_proto/tractor/v1/tracking_camera.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");
DEFINE_bool(detect_apriltags, false, "Detect apriltags.");

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetection;
using farm_ng_proto::tractor::v1::BUCKET_VIDEO_DATASETS;
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

namespace {
  bool ends_with(const std::string& s, const std::string& suffix) {
    if (s.length() < suffix.length()) {
      return false;
    }
    return (0 == s.compare(s.length() - suffix.length(), suffix.length(), suffix));
  }
} // namespace

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
        {bus_.GetName(), "/image$", "/apriltags$", "logger/status"});

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
    TrackingCameraCommand tracking_camera_command;
    if (configuration_.detect_apriltags()) {
      tracking_camera_command.mutable_record_start()->set_mode(
          TrackingCameraCommand::RecordStart::MODE_EVERY_APRILTAG_FRAME);
    } else {
      tracking_camera_command.mutable_record_start()->set_mode(
          TrackingCameraCommand::RecordStart::MODE_EVERY_FRAME);
    }
    RequestStartCapturing(bus_, tracking_camera_command);

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

  bool on_image(const EventPb& event) {
    Image image;
    if (!event.data().UnpackTo(&image)) {
      return false;
    }

    bool first_frame_for_camera = true;
    for (auto& entry : *status_.mutable_per_camera_num_frames()) {
      if (entry.camera_name() == image.camera_model().frame_name()) {
        entry.set_num_frames(entry.num_frames() + 1);
        first_frame_for_camera = false;
      }
    }
    if (first_frame_for_camera) {
        auto per_camera_num_frames = status_.add_per_camera_num_frames();
        per_camera_num_frames->set_camera_name(image.camera_model().frame_name());
        per_camera_num_frames->set_num_frames(1);
    }

    // TODO(ethanrublee | isherman): Remove (deprecated)
    status_.set_num_frames(status_.num_frames() + 1);

    return true;
  }

  bool on_apriltag_detection(const EventPb& event) {
    ApriltagDetection detection;
    if (!event.data().UnpackTo(&detection)) {
      return false;
    }

    bool first_time_seen = true;
    for (auto& entry : *status_.mutable_per_tag_id_num_frames()) {
      if (entry.tag_id() == detection.id()) {
        entry.set_num_frames(entry.num_frames() + 1);
        first_time_seen = false;
      }
    }
    if (first_time_seen) {
        auto per_tag_id_num_frames = status_.add_per_tag_id_num_frames();
        per_tag_id_num_frames->set_tag_id(detection.id());
        per_tag_id_num_frames->set_num_frames(1);
    }

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
    if (ends_with(event.name(), "/image") && on_image(event)) {
      return;
    }
    if (ends_with(event.name(), "/apriltags") && on_apriltag_detection(event)) {
      return;
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
  config.set_detect_apriltags(FLAGS_detect_apriltags);
  farm_ng::CaptureVideoDatasetProgram program(bus, config, FLAGS_interactive);
  return program.run();
}

int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
