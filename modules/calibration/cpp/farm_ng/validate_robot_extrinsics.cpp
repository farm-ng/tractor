#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/image_loader.h"

#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/calibration/validate_robot_extrinsics.pb.h"

#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"
#include "farm_ng/calibration/robot_hal_client.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");

DEFINE_string(name, "", "Log directory name.");
DEFINE_string(apriltag_rig, "", "Apriltag rig json file.");
DEFINE_string(hal_service_address, "", "Hal service address");

namespace farm_ng::calibration {

class MultiViewApriltagDetector {
 public:
  MultiViewApriltagDetector(perception::ApriltagConfig apriltag_config)
      : apriltag_config_(apriltag_config) {}

  perception::MultiViewApriltagDetections Detect(
      const google::protobuf::RepeatedPtrField<perception::Image>& images,
      google::protobuf::Timestamp stamp) {
    perception::MultiViewApriltagDetections multi_view_detections;
    for (const perception::Image& image : images) {
      std::string camera_frame_name = image.camera_model().frame_name();
      CHECK_GT(image.camera_model().image_width(), 1);

      if (!per_camera_model_.count(camera_frame_name)) {
        per_camera_model_[camera_frame_name] = image.camera_model();
        LOG(INFO) << image.camera_model().ShortDebugString();

        per_camera_detector_[camera_frame_name].reset(
            new perception::ApriltagDetector(image.camera_model(), nullptr,
                                             &apriltag_config_));
      }
      cv::Mat image_mat = image_loader_.LoadImage(image);
      cv::Mat depth_mat = image_loader_.LoadDepthmap(image);
      cv::Mat gray;
      if (image_mat.channels() == 3) {
        cv::cvtColor(image_mat, gray, cv::COLOR_BGR2GRAY);

      } else {
        CHECK(image_mat.channels() == 1);
        gray = image_mat;
      }
      auto tags = per_camera_detector_[camera_frame_name]->Detect(
          gray, depth_mat, stamp);

      tags.mutable_image()->CopyFrom(image);

      multi_view_detections.add_detections_per_view()->CopyFrom(tags);
      LOG(INFO) << camera_frame_name << " n tags: " << tags.detections_size();
    }
    return multi_view_detections;
  }

 private:
  std::map<std::string, perception::CameraModel> per_camera_model_;
  std::map<std::string, std::unique_ptr<perception::ApriltagDetector>>
      per_camera_detector_;

  perception::ImageLoader image_loader_;
  perception::ApriltagConfig apriltag_config_;
};

class ValidateRobotExtrinsicsProgram {
 public:
  ValidateRobotExtrinsicsProgram(
      core::EventBus& bus, bool interactive,
      calibration::ValidateRobotExtrinsicsConfiguration configuration)
      : bus_(bus),
        timer_(bus_.get_io_service()),
        configuration_(configuration) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName()});
    bus_.GetEventSignal()->connect(
        std::bind(&ValidateRobotExtrinsicsProgram::on_event, this,
                  std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  void send_status() {
    bus_.Send(core::MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&ValidateRobotExtrinsicsProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const core::Event& event) {
    ValidateRobotExtrinsicsConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    set_configuration(configuration);
    return true;
  }

  void set_configuration(
      const calibration::ValidateRobotExtrinsicsConfiguration& configuration) {
    LOG(INFO) << configuration.ShortDebugString();
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    status_.mutable_configuration()->CopyFrom(configuration_);
    send_status();
  }

  void on_event(const core::Event& event) {
    LOG(INFO) << "Event: " << event.ShortDebugString();
    if (on_configuration(event)) {
      return;
    }
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {});
    ValidateRobotExtrinsicsResult program_result;
    program_result.mutable_stamp_begin()->CopyFrom(core::MakeTimestampNow());
    program_result.mutable_configuration()->CopyFrom(configuration_);
    perception::ApriltagRig apriltag_rig =
        core::ReadProtobufFromResource<perception::ApriltagRig>(
            configuration_.apriltag_rig());

    perception::ApriltagConfig apriltag_config;
    AddApriltagRigToApriltagConfig(apriltag_rig, &apriltag_config);

    MultiViewApriltagDetector detector(apriltag_config);

    std::string log_path = (core::GetBucketRelativePath(core::BUCKET_LOGS) /
                            boost::filesystem::path(configuration_.name()))
                               .string();

    core::SetArchivePath(log_path);
    auto resource_path = farm_ng::core::GetUniqueArchiveResource(
        "events", "log", "application/farm_ng.eventlog.v1");
    program_result.mutable_event_log()->CopyFrom(resource_path.first);
    core::EventLogWriter log_writer(resource_path.second);

    farm_ng::calibration::RobotHalClient client(
        configuration_.hal_service_address());

    CalibratedCaptureRequest capture_request;
    log_writer.Write(core::MakeEvent("capture/request", capture_request));

    auto [capture_status, capture_response] =
        client.CalibratedCaptureSync(capture_request);
    if (!capture_status.ok()) {
      LOG(ERROR) << capture_status.error_message();
      return -1;
    }
    // Transform images to disk for logging purposes.
    for (perception::Image& image : *capture_response.mutable_images()) {
      perception::ImageResourceDataToPath(&image);
    }
    for (auto pose : capture_response.workspace_poses()) {
      LOG(INFO) << pose.ShortDebugString();
    }

    log_writer.Write(core::MakeEvent("capture/response", capture_response));

    auto multiview_detections =
        detector.Detect(capture_response.images(), capture_response.stamp());

    log_writer.Write(
        core::MakeEvent("multivew_detections", multiview_detections));

    auto result = EstimateMultiViewCameraRigPoseApriltagRig(
        capture_response.camera_rig(), apriltag_rig, multiview_detections);

    if (!result) {
      LOG(ERROR)
          << "Could not estimate the pose of the apriltag_rig in the capture.";
      return -1;
    }
    auto [pose, rmse, tag_stats] = *result;
    ApriltagRigPoseEstimateRequest estimate_request;
    estimate_request.set_capture_id(capture_response.capture_id());
    estimate_request.mutable_stamp()->CopyFrom(capture_response.stamp());
    estimate_request.mutable_apriltag_rig()->CopyFrom(apriltag_rig);
    estimate_request.mutable_poses()->CopyFrom(
        capture_response.workspace_poses());
    estimate_request.add_poses()->CopyFrom(pose);

    for (auto pose : estimate_request.poses()) {
      LOG(INFO) << pose.ShortDebugString();
    }

    estimate_request.set_rmse(rmse);
    estimate_request.mutable_multi_view_detections()->CopyFrom(
        multiview_detections);
    for (auto stats : tag_stats) {
      estimate_request.add_tag_stats()->CopyFrom(stats);
    }

    log_writer.Write(core::MakeEvent("estimate/request", estimate_request));
    program_result.mutable_estimate()->CopyFrom(estimate_request);
    status_.mutable_result()->CopyFrom(program_result);
    send_status();

    auto [estimate_status, estimate_response] =
        client.ApriltagRigPoseEstimateSync(estimate_request);
    if (!estimate_status.ok()) {
      LOG(ERROR) << estimate_status.error_message();
      return -1;
    }
    log_writer.Write(core::MakeEvent("estimate/response", estimate_response));
    program_result.mutable_stamp_end()->CopyFrom(core::MakeTimestampNow());
    core::ArchiveProtobufAsJsonResource("validation_result", program_result);

    status_.mutable_result()->CopyFrom(program_result);
    send_status();
    return 0;
  }

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;
  calibration::ValidateRobotExtrinsicsConfiguration configuration_;
  calibration::ValidateRobotExtrinsicsStatus status_;
};

}  // namespace farm_ng::calibration

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::ValidateRobotExtrinsicsConfiguration configuration;
  if (!FLAGS_interactive && FLAGS_hal_service_address.empty()) {
    LOG(ERROR) << "Please specify hal_service_address";
    return -1;
  }
  configuration.set_hal_service_address(FLAGS_hal_service_address);
  if (!FLAGS_interactive && FLAGS_apriltag_rig.empty()) {
    LOG(ERROR) << "Please specify apriltag_rig";
    return -1;
  }
  configuration.mutable_apriltag_rig()->set_path(FLAGS_apriltag_rig);
  configuration.mutable_apriltag_rig()->set_content_type(
      farm_ng::core::ContentTypeProtobufJson<
          farm_ng::perception::ApriltagRig>());
  if (!FLAGS_interactive && FLAGS_name.empty()) {
    LOG(ERROR) << "Please specify name";
    return -1;
  }
  configuration.set_name(FLAGS_name);

  farm_ng::calibration::ValidateRobotExtrinsicsProgram program(
      bus, FLAGS_interactive, configuration);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup"; }

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
