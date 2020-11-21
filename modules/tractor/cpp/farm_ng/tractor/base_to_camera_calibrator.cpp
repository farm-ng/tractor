#include "farm_ng/tractor/base_to_camera_calibrator.h"

#include <ceres/ceres.h>

#include <google/protobuf/util/time_util.h>

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"
#include "farm_ng/perception/time_series.h"

#include "farm_ng/tractor/tractor.pb.h"

typedef farm_ng::core::Event EventPb;
using farm_ng::calibration::AddSE3ParameterBlockSubsetTranslation;
using farm_ng::calibration::BaseToCameraInitialization;
using farm_ng::calibration::BaseToCameraModel;
using farm_ng::calibration::EstimateCameraPoseRig;
using farm_ng::calibration::LocalParameterizationAbs;
using farm_ng::calibration::MonocularApriltagRigModel;
using farm_ng::calibration::SolverStatus;
using farm_ng::calibration::TractorStartPoseTractorEnd;
using farm_ng::calibration::ViewDirection;
using farm_ng::calibration::ViewDirection_Name;
using farm_ng::calibration::ViewInitialization;
using farm_ng::core::EventBus;
using farm_ng::core::EventLogReader;
using farm_ng::core::MakeEvent;
using farm_ng::core::ReadProtobufFromResource;
using farm_ng::core::Resource;
using farm_ng::perception::ApriltagDetections;
using farm_ng::perception::NamedSE3Pose;
using farm_ng::perception::SE3Pose;
using farm_ng::perception::SophusToProto;
using farm_ng::perception::StartsWith;
using farm_ng::perception::TimeSeries;
using farm_ng::tractor::CopyTractorStateToWheelState;
using Sophus::SE3d;

namespace farm_ng {
namespace tractor {

std::vector<int> ViewInitializationToConstSubset(
    const ViewInitialization& view_initialization) {
  std::vector<int> const_subset;
  if (view_initialization.x().constant()) {
    const_subset.push_back(0);
  }
  if (view_initialization.y().constant()) {
    const_subset.push_back(1);
  }
  if (view_initialization.z().constant()) {
    const_subset.push_back(2);
  }
  return const_subset;
}

void ViewInitializationToNamedSE3Pose(
    const ViewInitialization& view_initialization,
    const std::string& base_frame_name, const std::string& camera_frame_name,
    NamedSE3Pose* base_pose_camera_pb) {
  SE3d base_pose_camera;
  switch (view_initialization.view_direction()) {
    case ViewDirection::VIEW_DIRECTION_FRONT:
      base_pose_camera = Sophus::SE3d::rotZ(-M_PI / 2.0) *
                         Sophus::SE3d::rotX(M_PI / 2.0) *
                         Sophus::SE3d::rotY(M_PI) * Sophus::SE3d::rotZ(M_PI);
      break;
    case ViewDirection::VIEW_DIRECTION_FRONT_INVERTED:
      base_pose_camera = Sophus::SE3d::rotZ(-M_PI / 2.0) *
                         Sophus::SE3d::rotX(M_PI / 2.0) *
                         Sophus::SE3d::rotY(M_PI);
      break;
    default:
      LOG(FATAL) << ViewDirection_Name(view_initialization.view_direction())
                 << "Not handled.";
  }
  base_pose_camera.translation().x() = view_initialization.x().value();
  base_pose_camera.translation().y() = view_initialization.y().value();
  base_pose_camera.translation().z() = view_initialization.z().value();
  SophusToProto(base_pose_camera, base_frame_name, camera_frame_name,
                base_pose_camera_pb);
}

struct BasePoseCameraCostFunctor {
  BasePoseCameraCostFunctor(const BaseToCameraModel::Sample& sample)
      : sample_(sample) {
    SE3d camera_pose_rig_end;
    SE3d camera_pose_rig_start;
    ProtoToSophus(sample.camera_pose_rig_start().a_pose_b(),
                  &camera_pose_rig_start);
    ProtoToSophus(sample.camera_pose_rig_end().a_pose_b(),
                  &camera_pose_rig_end);
    camera_end_pose_camera_start_ =
        camera_pose_rig_end * camera_pose_rig_start.inverse();
  }

  template <class T>
  bool operator()(T const* const raw_base_pose_camera,
                  T const* const raw_base_radius,
                  T const* const raw_base_wheel_baseline,
                  T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const base_pose_camera(
        raw_base_pose_camera);

    const Sophus::SE3<T> camera_end_pose_camera_start(
        camera_end_pose_camera_start_.cast<T>());
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);

    Sophus::SE3<T> tractor_start_pose_tractor_end =
        TractorStartPoseTractorEnd<T>(*raw_base_radius,
                                      *raw_base_wheel_baseline, sample_);

    auto tractor_start_pose_tractor_start =
        tractor_start_pose_tractor_end * base_pose_camera *
        camera_end_pose_camera_start * base_pose_camera.inverse();

    residuals = tractor_start_pose_tractor_start.log();
    return true;
  }
  BaseToCameraModel::Sample sample_;
  SE3d camera_end_pose_camera_start_;
};

class BaseToCameraIterationCallback : public ceres::IterationCallback {
 public:
  explicit BaseToCameraIterationCallback(const double* base_pose_camera,
                                         const double* wheel_radius,
                                         const double* wheel_baseline)

      : base_pose_camera_(base_pose_camera),
        wheel_radius_(wheel_radius),
        wheel_baseline_(wheel_baseline) {}

  ~BaseToCameraIterationCallback() {}

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary = ceres::IterationSummary()) {
    SE3Pose pose_pb;
    Eigen::Map<SE3d const> base_pose_camera(base_pose_camera_);
    SophusToProto(Eigen::Map<SE3d const>(base_pose_camera_), &pose_pb);
    LOG(INFO) << "\n[inches] base_pose_camera x: "
              << base_pose_camera.translation().x() / 0.0254
              << " y: " << base_pose_camera.translation().y() / 0.0254
              << " z: " << base_pose_camera.translation().z() / 0.0254
              << "\n[inches] wheel_radius: " << (*wheel_radius_) / 0.0254
              << " wheel_baseline: " << (*wheel_baseline_) / 0.0254
              << "\nbase_pose_camera Rotation:\n"
              << base_pose_camera.rotationMatrix();
    return ceres::SOLVER_CONTINUE;
  }

 private:
  const double* base_pose_camera_;
  const double* wheel_radius_;
  const double* wheel_baseline_;
};

BaseToCameraModel SolveBasePoseCamera(BaseToCameraModel model,
                                      BasePoseCameraSolverOptions options) {
  // base_pose_camera is a parameter block, modified by ceres.
  SE3d base_pose_camera;
  ProtoToSophus(model.base_pose_camera().a_pose_b(), &base_pose_camera);
  // wheel_* are parameter blocks, modified by ceres.
  double wheel_radius = model.wheel_radius();
  double wheel_baseline = model.wheel_baseline();

  ceres::Problem problem;

  // Here we hold Z constant, through the local parameterization.
  AddSE3ParameterBlockSubsetTranslation(
      &problem, base_pose_camera.data(),
      ViewInitializationToConstSubset(
          model.initialization().base_pose_camera()));

  if (options.hold_base_pose_camera_constant) {
    problem.SetParameterBlockConstant(base_pose_camera.data());
  }
  problem.AddParameterBlock(&wheel_radius, 1, new LocalParameterizationAbs(1));
  problem.AddParameterBlock(&wheel_baseline, 1,
                            new LocalParameterizationAbs(1));

  if (options.hold_base_parameters_constant ||
      model.initialization().wheel_baseline().constant()) {
    problem.SetParameterBlockConstant(&wheel_baseline);
  }

  if (options.hold_base_parameters_constant ||
      model.initialization().wheel_radius().constant()) {
    problem.SetParameterBlockConstant(&wheel_radius);
  }

  for (const auto& sample : model.samples()) {
    ceres::CostFunction* cost_function1 =
        new ceres::AutoDiffCostFunction<BasePoseCameraCostFunctor, 6,
                                        SE3d::num_parameters, 1, 1>(
            new BasePoseCameraCostFunctor(sample));
    problem.AddResidualBlock(cost_function1, nullptr, base_pose_camera.data(),
                             &wheel_radius, &wheel_baseline);
  }

  BaseToCameraIterationCallback callback(base_pose_camera.data(), &wheel_radius,
                                         &wheel_baseline);
  // Set solver options (precision / method)
  ceres::Solver::Options solver_options;
  solver_options.gradient_tolerance = 1e-8;
  solver_options.function_tolerance = 1e-8;
  solver_options.parameter_tolerance = 1e-8;
  solver_options.max_num_iterations = 2000;
  solver_options.callbacks.push_back(&callback);

  // Solve
  ceres::Solver::Summary summary;
  solver_options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.update_state_every_iteration = true;
  ceres::Solve(solver_options, &problem, &summary);

  LOG(INFO) << summary.FullReport() << std::endl;
  double rmse = std::sqrt(summary.final_cost / summary.num_residuals);

  if (summary.IsSolutionUsable()) {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_CONVERGED);
  } else {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_FAILED);
  }

  model.set_rmse(rmse);
  model.set_wheel_radius(wheel_radius);
  model.set_wheel_baseline(wheel_baseline);
  SophusToProto(base_pose_camera,
                model.mutable_base_pose_camera()->mutable_a_pose_b());

  for (auto& sample : *model.mutable_samples()) {
    sample.mutable_odometry_trajectory_base()->Clear();
    SE3d odometry_pose_base;
    for (const auto& wheel_measurement : sample.wheel_measurements()) {
      auto base_pose_delta =
          TractorPoseDelta(wheel_radius, wheel_baseline, wheel_measurement);
      odometry_pose_base = odometry_pose_base * base_pose_delta;
      SophusToProto(odometry_pose_base, wheel_measurement.stamp(),
                    sample.mutable_odometry_trajectory_base()->add_a_poses_b());
    }

    sample.mutable_visual_odometry_trajectory_base()->Clear();

    SE3d camera_pose_rig_start;
    ProtoToSophus(sample.camera_pose_rig_start().a_pose_b(),
                  &camera_pose_rig_start);

    SophusToProto(
        SE3d(), sample.camera_pose_rig_start().a_pose_b().stamp(),
        sample.mutable_visual_odometry_trajectory_base()->add_a_poses_b());

    for (const auto& camera_pose_rig_pb :
         sample.camera_trajectory_rig().a_poses_b()) {
      SE3d camera_pose_rig;
      ProtoToSophus(camera_pose_rig_pb, &camera_pose_rig);
      SophusToProto(
          base_pose_camera * camera_pose_rig_start * camera_pose_rig.inverse() *
              base_pose_camera.inverse(),
          camera_pose_rig_pb.stamp(),
          sample.mutable_visual_odometry_trajectory_base()->add_a_poses_b());
    }
    {
      SE3d camera_pose_rig_end;
      ProtoToSophus(sample.camera_pose_rig_end().a_pose_b(),
                    &camera_pose_rig_end);
      SophusToProto(
          base_pose_camera * camera_pose_rig_start *
              camera_pose_rig_end.inverse() * base_pose_camera.inverse(),
          sample.camera_pose_rig_end().a_pose_b().stamp(),
          sample.mutable_visual_odometry_trajectory_base()->add_a_poses_b());
    }
  }
  LOG(INFO) << "root mean of residual error: " << rmse;
  callback();
  return model;
}

void CopyTractorStateToWheelState(
    const TractorState& tractor_state,
    BaseToCameraModel::WheelMeasurement* wheel_measurement) {
  wheel_measurement->set_wheel_velocity_rads_left(
      tractor_state.wheel_velocity_rads_left());

  wheel_measurement->set_wheel_velocity_rads_right(
      tractor_state.wheel_velocity_rads_right());

  wheel_measurement->set_dt(tractor_state.dt());

  wheel_measurement->mutable_stamp()->CopyFrom(tractor_state.stamp());
}

BaseToCameraModel InitialBaseToCameraModelFromEventLog(
    const BaseToCameraInitialization& initialization,
    const Resource& event_log_resource, const Resource& apriltag_rig_resource) {
  auto rig_model = ReadProtobufFromResource<MonocularApriltagRigModel>(
      apriltag_rig_resource);

  EventLogReader log_reader(event_log_resource);
  BaseToCameraModel model;
  model.mutable_initialization()->CopyFrom(initialization);
  model.set_wheel_radius(initialization.wheel_radius().value());
  model.set_wheel_baseline(initialization.wheel_baseline().value());
  ViewInitializationToNamedSE3Pose(
      initialization.base_pose_camera(), "tractor/base",
      rig_model.camera_frame_name(), model.mutable_base_pose_camera());

  model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);
  BaseToCameraModel::Sample sample;

  bool has_start = false;

  // Set this to add the april tag trajectory to sample, useful for
  // visualization of pose error.
  bool full_apriltag_trajectory = true;

  TimeSeries<BaseToCameraModel::WheelMeasurement> wheel_measurement_series;
  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
      TractorState tractor_state;
      if (event.data().UnpackTo(&tractor_state)) {
        BaseToCameraModel::WheelMeasurement wheel_measurement;
        CopyTractorStateToWheelState(tractor_state, &wheel_measurement);
        wheel_measurement_series.insert(wheel_measurement);
      } else {
        bool calibration_sample = StartsWith(event.name(), "calibrator");
        if (!calibration_sample && !full_apriltag_trajectory) {
          continue;
        }
        ApriltagDetections detections;
        if (!event.data().UnpackTo(&detections)) {
          continue;
        }
        if (detections.detections_size() == 0) {
          continue;
        }
        // LOG(INFO) << event.ShortDebugString();
        Sophus::optional<NamedSE3Pose> o_camera_pose_rig =
            EstimateCameraPoseRig(rig_model.rig(), detections);
        if (!o_camera_pose_rig) {
          continue;
        }
        // set our event stamp as the pose stamp.
        o_camera_pose_rig->mutable_a_pose_b()->mutable_stamp()->CopyFrom(
            event.stamp());
        if (!has_start && calibration_sample) {
          sample.mutable_camera_trajectory_rig()->set_frame_a(
              o_camera_pose_rig->frame_a());
          sample.mutable_camera_trajectory_rig()->set_frame_b(
              o_camera_pose_rig->frame_b());

          sample.mutable_camera_pose_rig_start()->CopyFrom(*o_camera_pose_rig);
          has_start = true;
        } else if (has_start && !calibration_sample &&
                   full_apriltag_trajectory) {
          CHECK_EQ(sample.camera_trajectory_rig().frame_a(),
                   o_camera_pose_rig->frame_a());
          CHECK_EQ(sample.camera_trajectory_rig().frame_b(),
                   o_camera_pose_rig->frame_b());
          sample.mutable_camera_trajectory_rig()->add_a_poses_b()->CopyFrom(
              o_camera_pose_rig->a_pose_b());
        } else if (has_start && calibration_sample) {
          sample.mutable_camera_pose_rig_end()->CopyFrom(*o_camera_pose_rig);

          auto begin_end = wheel_measurement_series.find_range(
              sample.camera_pose_rig_start().a_pose_b().stamp(),
              sample.camera_pose_rig_end().a_pose_b().stamp());
          CHECK(begin_end.first != begin_end.second)
              << "Couldn't find any wheel measurements in the given range: "
              << sample.camera_pose_rig_start()
                     .a_pose_b()
                     .stamp()
                     .ShortDebugString()
              << " - "
              << sample.camera_pose_rig_end()
                     .a_pose_b()
                     .stamp()
                     .ShortDebugString();
          auto begin_dt_millisecond =
              google::protobuf::util::TimeUtil::DurationToMilliseconds(
                  (begin_end.first->stamp() -
                   sample.camera_pose_rig_start().a_pose_b().stamp()));
          // this consistently finds a range that is within ~10 milliseconds of
          // the beginning or end.
          while (begin_end.first != begin_end.second) {
            sample.add_wheel_measurements()->CopyFrom((*begin_end.first++));
          }
          auto end_dt_millisecond =
              google::protobuf::util::TimeUtil::DurationToMilliseconds(
                  (sample.wheel_measurements().rbegin()->stamp() -
                   sample.camera_pose_rig_end().a_pose_b().stamp()));
          model.add_samples()->CopyFrom(sample);
          LOG(INFO) << "n wheel measurments: "
                    << sample.wheel_measurements().size()
                    << " begin dt ms: " << begin_dt_millisecond
                    << " end dt ms: " << end_dt_millisecond;
          sample.Clear();
          sample.mutable_camera_pose_rig_start()->CopyFrom(*o_camera_pose_rig);
          sample.mutable_camera_trajectory_rig()->set_frame_a(
              o_camera_pose_rig->frame_a());
          sample.mutable_camera_trajectory_rig()->set_frame_b(
              o_camera_pose_rig->frame_b());
        }
      }
    } catch (std::runtime_error& e) {
      break;
    }
  }
  return model;
}

}  // namespace tractor
}  // namespace farm_ng
