#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/imgproc.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/image.pb.h"
#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"

#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/calibration/capture_robot_extrinsics_dataset.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(dataset, "", "CaptureRobotExtrinsicsResult");
DEFINE_string(initial, "", "Start from an initial model");
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::ReadProtobufFromJsonFile;
using farm_ng::perception::Image;
using farm_ng::perception::NamedSE3Pose;
typedef farm_ng::core::Event EventPb;

namespace farm_ng::calibration {

RobotExtrinsicsModel RobotExtrinsicsModelFromDatasetResult(
    const CaptureRobotExtrinsicsDatasetResult& dataset_result) {
  perception::ApriltagConfig apriltag_config;
  RobotExtrinsicsModel model;
  model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);

  model.set_workspace_frame_name(
      dataset_result.configuration().workspace_frame_name());
  model.set_base_frame_name(dataset_result.configuration().base_frame_name());
  model.set_link_frame_name(dataset_result.configuration().link_frame_name());

  if (dataset_result.configuration().has_link_tag_rig()) {
    AddApriltagRigToApriltagConfig(
        dataset_result.configuration().link_tag_rig(), &apriltag_config);

    model.mutable_link_tag_rig()->CopyFrom(
        dataset_result.configuration().link_tag_rig());
  }

  if (dataset_result.configuration().has_base_tag_rig()) {
    AddApriltagRigToApriltagConfig(
        dataset_result.configuration().base_tag_rig(), &apriltag_config);

    model.mutable_base_tag_rig()->CopyFrom(
        dataset_result.configuration().base_tag_rig());
  }

  if (dataset_result.configuration().has_base_camera_rig()) {
    model.mutable_base_camera_rig()->CopyFrom(
        dataset_result.configuration().base_camera_rig());
    LOG(INFO)
        << dataset_result.configuration().base_camera_rig().ShortDebugString();
  }

  if (dataset_result.configuration().has_link_camera_rig()) {
    model.mutable_link_camera_rig()->CopyFrom(
        dataset_result.configuration().link_camera_rig());
  }

  std::map<std::string, perception::CameraModel> per_camera_model;
  std::map<std::string, std::unique_ptr<perception::ApriltagDetector>>
      per_camera_detector;

  core::EventLogReader log_reader(dataset_result.dataset());
  perception::ImageLoader image_loader;
  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (const std::runtime_error& e) {
      LOG(INFO) << e.what();
      break;
    }
    CapturePoseResponse pose_response;
    if (event.data().UnpackTo(&pose_response)) {
      RobotExtrinsicsModel::Measurement* measurement = model.add_measurements();

      measurement->mutable_poses()->CopyFrom(pose_response.poses());
      measurement->mutable_joint_states()->CopyFrom(
          pose_response.joint_states());

      for (Image image : pose_response.images()) {
        std::string camera_frame_name = image.camera_model().frame_name();
        if (image.camera_model().image_width() == 1) {
          // TODO remove this hack as after dataset is fixed.
          LOG_FIRST_N(WARNING, 10)
              << "Malformed camera model, defaulting to width=1920 "
                 "height=1080";
          perception::CameraModel* camera_model = image.mutable_camera_model();
          camera_model->set_image_width(1920);
          camera_model->set_image_height(1080);
        }

        if (!per_camera_model.count(camera_frame_name)) {
          per_camera_model[camera_frame_name] = image.camera_model();
          LOG(INFO) << image.camera_model().ShortDebugString();

          per_camera_detector[camera_frame_name].reset(
              new perception::ApriltagDetector(image.camera_model(), nullptr,
                                               &apriltag_config));
        }
        cv::Mat image_mat = image_loader.LoadImage(image);
        cv::Mat depth_mat = image_loader.LoadDepthmap(image);
        cv::Mat gray;
        if (image_mat.channels() == 3) {
          cv::cvtColor(image_mat, gray, cv::COLOR_BGR2GRAY);

        } else {
          CHECK(image_mat.channels() == 1);
          gray = image_mat;
        }
        auto tags = per_camera_detector[camera_frame_name]->Detect(
            gray, depth_mat, event.stamp());
        measurement->mutable_multi_view_detections()
            ->add_detections_per_view()
            ->CopyFrom(tags);
        LOG(INFO) << camera_frame_name << " n tags: " << tags.detections_size();
      }
    }
  }
  return model;
}

class CalibrateRobotExtrinsicsProgram {
 public:
  CalibrateRobotExtrinsicsProgram(core::EventBus& bus, bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      // status_.mutable_input_required_resource()->CopyFrom(resource);
    } else {
      // set_configuration(resource);
    }
    bus_.AddSubscriptions({bus_.GetName()});
    bus_.GetEventSignal()->connect(
        std::bind(&CalibrateRobotExtrinsicsProgram::on_event, this,
                  std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  void send_status() {
    // bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&CalibrateRobotExtrinsicsProgram::on_timer,
                                this, std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    core::Resource configuration_resource;
    if (!event.data().UnpackTo(&configuration_resource)) {
      return false;
    }
    LOG(INFO) << configuration_resource.ShortDebugString();
    set_configuration(configuration_resource);
    return true;
  }

  void set_configuration(const core::Resource& resource) { send_status(); }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

  int run() {
    // while (status_.has_input_required_resource()) {
    //      bus_.get_io_service().run_one();
    //}

    WaitForServices(bus_, {});

    core::Resource dataset_resource;
    dataset_resource.set_path(FLAGS_dataset);
    dataset_resource.set_content_type(
        core::ContentTypeProtobufJson<CaptureRobotExtrinsicsDatasetResult>());
    CaptureRobotExtrinsicsDatasetResult dataset_result =
        core::ReadProtobufFromResource<CaptureRobotExtrinsicsDatasetResult>(
            dataset_resource);
    auto output_dir =
        boost::filesystem::path(dataset_result.dataset().path()).parent_path();

    // Output under the same directory as the dataset.
    core::SetArchivePath((output_dir / "robot_extrinsics").string());

    RobotExtrinsicsModel model;
    core::Resource initial_resource;

    if (FLAGS_initial.empty()) {
      model = RobotExtrinsicsModelFromDatasetResult(dataset_result);

      initial_resource =
          core::ArchiveProtobufAsBinaryResource("initial", model);
      LOG(INFO) << "Wrote initial model to blobstore: "
                << initial_resource.path();
    } else {
      initial_resource.set_path(FLAGS_initial);
      initial_resource.set_content_type(
          core::ContentTypeProtobufBinary<RobotExtrinsicsModel>());
      model = core::ReadProtobufFromResource<RobotExtrinsicsModel>(
          initial_resource);
      LOG(INFO) << model.ShortDebugString();
    }

    return 0;
  }

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;
};

}  // namespace farm_ng::calibration

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::CalibrateRobotExtrinsicsProgram program(
      bus, FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup"; }

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
