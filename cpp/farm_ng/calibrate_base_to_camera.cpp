#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/event_log_reader.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"

#include "farm_ng/calibration/base_to_camera_calibrator.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrate_apriltag_rig.pb.h"
#include "farm_ng_proto/tractor/v1/calibrate_base_to_camera.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/capture_calibration_dataset.pb.h"

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::BaseToCameraModel;
using farm_ng_proto::tractor::v1::CalibrateApriltagRigResult;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraConfiguration;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraResult;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraStatus;

using farm_ng_proto::tractor::v1::CaptureCalibrationDatasetResult;
using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::SolverStatus;
using farm_ng_proto::tractor::v1::SolverStatus_Name;
using farm_ng_proto::tractor::v1::Subscription;
using farm_ng_proto::tractor::v1::ViewDirection;
using farm_ng_proto::tractor::v1::ViewDirection_Name;
using farm_ng_proto::tractor::v1::ViewDirection_Parse;

DEFINE_bool(interactive, false, "receive program args via eventbus");

DEFINE_string(calibration_dataset, "",
              "The path to a serialized CaptureCalibrationDatasetResult");
DEFINE_string(apriltag_rig_result, "",
              "The path to a serialized ApriltagRigCalibrationResult");
DEFINE_double(wheel_baseline, 42, "The wheel baseline parameter");
DEFINE_bool(wheel_baseline_constant, false,
            "Hold the wheel baseline parameter constant");
DEFINE_double(wheel_radius, 8.5, "The wheel radius parameter");
DEFINE_bool(wheel_radius_constant, false,
            "Hold the wheel radius parameter constant");
DEFINE_double(
    base_pose_camera_tx, -12.0,
    "The x component of translation to use for base_to_camera initialization");
DEFINE_bool(base_pose_camera_tx_constant, false,
            "Hold the base_pose_camera_tx parameter constant");
DEFINE_double(
    base_pose_camera_ty, 42 / 2.0,
    "The y component of translation to use for base_to_camera initialization");
DEFINE_bool(base_pose_camera_ty_constant, false,
            "Hold the base_pose_camera_ty parameter constant");
DEFINE_double(
    base_pose_camera_tz, 48.0,
    "The z component of translation to use for base_to_camera initialization");
DEFINE_bool(base_pose_camera_tz_constant, true,
            "Hold the base_pose_camera_tz parameter constant");
DEFINE_string(
    camera_direction, ViewDirection_Name(ViewDirection::VIEW_DIRECTION_FRONT),
    "The orientation of the camera, used for base_to_camera initialization. "
    "Must be a valid ViewDirection enum key.");
DEFINE_string(name, "base_to_camera", "Name of the calibration.");

namespace farm_ng {
class CalibrateBaseToCameraProgram {
 public:
  CalibrateBaseToCameraProgram(EventBus& bus,
                               CalibrateBaseToCameraConfiguration configuration,
                               bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({"^" + bus_.GetName() + "/"});
    bus_.GetEventSignal()->connect(std::bind(
        &CalibrateBaseToCameraProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    auto dataset_result =
        ReadProtobufFromResource<CaptureCalibrationDatasetResult>(
            configuration_.calibration_dataset());

    auto rig_result = ReadProtobufFromResource<CalibrateApriltagRigResult>(
        configuration_.apriltag_rig_result());

    auto output_dir =
        boost::filesystem::path(dataset_result.dataset().path()).parent_path();

    // Output under the same directory as the dataset.
    SetArchivePath((output_dir / "base_to_camera").string());

    BaseToCameraInitialization initialization = configuration_.initialization();

    BaseToCameraModel model = InitialBaseToCameraModelFromEventLog(
        initialization, dataset_result.dataset(),
        rig_result.monocular_apriltag_rig_solved());

    CalibrateBaseToCameraResult result;
    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_base_to_camera_model_initial()->CopyFrom(
        ArchiveProtobufAsBinaryResource("initial", model));
    result.set_solver_status(model.solver_status());
    result.set_rmse(model.rmse());
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    // if (log.has_recording()) {
    //      result.mutable_event_log()->set_path(log.recording().archive_path());
    //}
    status_.mutable_result()->CopyFrom(
        ArchiveProtobufAsJsonResource("initial_result", result));
    send_status();

    BasePoseCameraSolverOptions options;

    // First hold our base parameters constant, so that the camera pose is
    // solved for, this seems to be stable even with very poor initialization.
    options.hold_base_parameters_constant = true;
    auto solved_model = SolveBasePoseCamera(model, options);

    if (solved_model.solver_status() != SolverStatus::SOLVER_STATUS_CONVERGED) {
      LOG(FATAL) << "Failed to solve the initial base_pose_camera model: "
                 << SolverStatus_Name(solved_model.solver_status());
    }
    // Now allow the solver to adjust everything, since we have an optimized
    // camera pose.  Keep in mind the constantness from the configuration will
    // take precident.
    options.hold_base_parameters_constant = false;
    solved_model = SolveBasePoseCamera(solved_model, options);
    result.mutable_base_to_camera_model_solved()->CopyFrom(
        ArchiveProtobufAsBinaryResource("solved", solved_model));
    result.set_solver_status(solved_model.solver_status());
    result.set_rmse(solved_model.rmse());
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());

    // TODO some how save the result in the archive directory as well, so its
    // self contained.
    ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        BucketId::kBaseToCameraModels, configuration_.name(), result));
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
    timer_.async_wait(std::bind(&CalibrateBaseToCameraProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    CalibrateBaseToCameraConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CalibrateBaseToCameraConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    CHECK(event.name().rfind(bus_.GetName() + "/", 0) == 0);
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CalibrateBaseToCameraConfiguration configuration_;
  CalibrateBaseToCameraStatus status_;
  CalibrateBaseToCameraResult result_;
};

}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) { LOG(INFO) << "Cleanup."; }

int Main(farm_ng::EventBus& bus) {
  CalibrateBaseToCameraConfiguration config;
  config.mutable_calibration_dataset()->set_path(FLAGS_calibration_dataset);
  config.mutable_calibration_dataset()->set_content_type(
      farm_ng::ContentTypeProtobufJson<CaptureCalibrationDatasetResult>());

  config.mutable_apriltag_rig_result()->set_path(FLAGS_apriltag_rig_result);
  config.mutable_apriltag_rig_result()->set_content_type(
      farm_ng::ContentTypeProtobufJson<CalibrateApriltagRigResult>());
  auto initialization = config.mutable_initialization();
  initialization->mutable_wheel_baseline()->set_value(FLAGS_wheel_baseline *
                                                      0.0254);
  initialization->mutable_wheel_baseline()->set_constant(
      FLAGS_wheel_baseline_constant);
  initialization->mutable_wheel_radius()->set_value(FLAGS_wheel_radius *
                                                    0.0254);
  initialization->mutable_wheel_radius()->set_constant(
      FLAGS_wheel_radius_constant);

  initialization->mutable_base_pose_camera()->mutable_x()->set_value(
      FLAGS_base_pose_camera_tx * 0.0254);
  initialization->mutable_base_pose_camera()->mutable_x()->set_constant(
      FLAGS_base_pose_camera_tx_constant);
  initialization->mutable_base_pose_camera()->mutable_y()->set_value(
      FLAGS_base_pose_camera_ty * 0.0254);
  initialization->mutable_base_pose_camera()->mutable_y()->set_constant(
      FLAGS_base_pose_camera_ty_constant);
  initialization->mutable_base_pose_camera()->mutable_z()->set_value(
      FLAGS_base_pose_camera_tz * 0.0254);
  initialization->mutable_base_pose_camera()->mutable_z()->set_constant(
      FLAGS_base_pose_camera_tz_constant);
  ViewDirection camera_direction;
  CHECK(ViewDirection_Parse(FLAGS_camera_direction, &camera_direction));
  initialization->mutable_base_pose_camera()->set_view_direction(
      camera_direction);
  config.set_name(FLAGS_name);

  farm_ng::CalibrateBaseToCameraProgram program(bus, config, FLAGS_interactive);
  return program.run();
}
int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
