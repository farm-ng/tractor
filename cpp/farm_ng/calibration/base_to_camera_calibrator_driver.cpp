#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ceres/ceres.h>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/time_util.h>

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/local_parameterization_se3.h"
#include "farm_ng/calibration/pose_utils.h"
#include "farm_ng/event_log_reader.h"
#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/tractor.pb.h"

DEFINE_string(log, "", "Path to log file, recorded with farm-ng-ipc-logger");
DEFINE_string(
    rig_calibration, "",
    "Path to a rig calibration file, recorded with farm-ng-ipc-logger");

DEFINE_string(out_archive, "default",
              "When running from a log, what archive name should we write to?");

DEFINE_string(camera_direction, "front-upside-down",
              "Which direction is camera facing?  Choose one of: front, "
              "front-upside-down");

DEFINE_double(wheel_radius, 8.5, "Wheel radius in inches.");
DEFINE_double(wheel_baseline, 42, "Wheel baseline in inches.");

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::BaseToCameraModel;
using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::TractorState;

namespace farm_ng {
MonocularApriltagRigModel ReadMonocularApriltagRigModelFromDisk(
    const std::string json_file) {
  std::ifstream rig_in(json_file);
  CHECK(rig_in) << "Could not open json_file: " << json_file;
  std::string rig_json_str((std::istreambuf_iterator<char>(rig_in)),
                           std::istreambuf_iterator<char>());

  CHECK(!rig_json_str.empty()) << "Did not load any text from: " << json_file;
  google::protobuf::util::JsonParseOptions options;

  MonocularApriltagRigModel rig_model;
  auto status = google::protobuf::util::JsonStringToMessage(
      rig_json_str, &rig_model, options);
  CHECK(status.ok()) << status;

  return rig_model;
}

class LocalParameterizationAbs : public ceres::LocalParameterization {
 public:
  LocalParameterizationAbs(int size) : size_(size) {}

  virtual ~LocalParameterizationAbs() {}

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
    ceres::VectorRef(x_plus_delta, size_) =
        (ceres::ConstVectorRef(x, size_) + ceres::ConstVectorRef(delta, size_))
            .cwiseAbs();

    return true;
  }

  bool ComputeJacobian(const double* x, double* jacobian) const {
    ceres::MatrixRef(jacobian, size_, size_).setIdentity();
    return true;
  }

  virtual int GlobalSize() const { return size_; }

  virtual int LocalSize() const { return size_; }
  int size_;
};

template <typename T>
static Sophus::SE3<T> TractorPoseDelta(
    const Eigen::Matrix<T, 2, 1>& base_parameters,
    const BaseToCameraModel::WheelMeasurement wheel_measurement) {
  const T& wheel_radius = base_parameters[0];
  const T& wheel_baseline = base_parameters[1];
  T vel_left(wheel_measurement.wheel_velocity_rads_left());
  T vel_right(wheel_measurement.wheel_velocity_rads_right());
  T dt(wheel_measurement.dt());
  T v = T(wheel_radius / 2.0) * (vel_left + vel_right);
  T w = (wheel_radius / wheel_baseline) * (vel_right - vel_left);
  Eigen::Matrix<T, 6, 1> x;
  x << v * dt, T(0), T(0), T(0), T(0), w * dt;
  return Sophus::SE3<T>::exp(x);
}
template <typename T>
Sophus::SE3<T> TractorStartPoseTractorEnd(
    const Eigen::Matrix<T, 2, 1>& base_params,
    const BaseToCameraModel::Sample& sample) {
  Sophus::SE3<T> tractor_start_pose_tractor_end =
      Sophus::SE3d::rotZ(0).cast<T>();
  for (const auto& wheel_state : sample.wheel_measurements()) {
    tractor_start_pose_tractor_end =
        tractor_start_pose_tractor_end *
        TractorPoseDelta<T>(base_params, wheel_state);
  }
  return tractor_start_pose_tractor_end;
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
                  T const* const raw_base_params, T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const base_pose_camera(
        raw_base_pose_camera);
    Eigen::Map<Eigen::Matrix<T, 2, 1> const> const base_params(raw_base_params);
    const Sophus::SE3<T> camera_end_pose_camera_start(
        camera_end_pose_camera_start_.cast<T>());
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);

    Sophus::SE3<T> tractor_start_pose_tractor_end =
        TractorStartPoseTractorEnd<T>(base_params, sample_);

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
  explicit BaseToCameraIterationCallback(const SE3d* base_pose_camera,
                                         const Eigen::Vector2d* base_parameters)
      : base_pose_camera_(base_pose_camera),
        base_parameters_(base_parameters) {}

  ~BaseToCameraIterationCallback() {}

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary = ceres::IterationSummary()) {
    LOG(INFO) << "base_pose_camera:\n"
              << base_pose_camera_->matrix3x4()
              << "\nwheel_radius (inches): " << (*base_parameters_)[0] / 0.0254
              << " wheel_baseline (inches): "
              << (*base_parameters_)[1] / 0.0254;
    return ceres::SOLVER_CONTINUE;
  }

 private:
  const SE3d* base_pose_camera_;
  const Eigen::Vector2d* base_parameters_;
};

BaseToCameraModel SolveBasePoseCamera(BaseToCameraModel model,
                                      bool hold_base_parameters_const) {
  SE3d base_pose_camera;
  ProtoToSophus(model.base_pose_camera().a_pose_b(), &base_pose_camera);
  Eigen::Vector2d base_parameters(model.wheel_radius(), model.wheel_baseline());

  ceres::Problem problem;

  problem.AddParameterBlock(base_pose_camera.data(), SE3d::num_parameters,
                            new LocalParameterizationSE3);

  problem.AddParameterBlock(base_parameters.data(), 2,
                            new LocalParameterizationAbs(2));

  if (hold_base_parameters_const) {
    problem.SetParameterBlockConstant(base_parameters.data());
  }
  for (const auto& sample : model.samples()) {
    ceres::CostFunction* cost_function1 =
        new ceres::AutoDiffCostFunction<BasePoseCameraCostFunctor, 6,
                                        Sophus::SE3d::num_parameters, 2>(
            new BasePoseCameraCostFunctor(sample));
    problem.AddResidualBlock(cost_function1, nullptr, base_pose_camera.data(),
                             base_parameters.data());
  }

  BaseToCameraIterationCallback callback(&base_pose_camera, &base_parameters);
  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-8;
  options.function_tolerance = 1e-8;
  options.parameter_tolerance = 1e-8;
  options.max_num_iterations = 2000;
  options.callbacks.push_back(&callback);

  // Solve
  ceres::Solver::Summary summary;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport() << std::endl;
  double rmse = std::sqrt(summary.final_cost / summary.num_residuals);

  if (summary.IsSolutionUsable()) {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_CONVERGED);
  } else {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_FAILED);
  }

  model.set_rmse(rmse);
  model.set_wheel_radius(base_parameters[0]);
  model.set_wheel_baseline(base_parameters[1]);
  SophusToProto(base_pose_camera,
                model.mutable_base_pose_camera()->mutable_a_pose_b());

  for (auto& sample : *model.mutable_samples()) {
    sample.mutable_odometry_trajectory_base()->Clear();
    SE3d odometry_pose_base;
    for (const auto& wheel_measurement : sample.wheel_measurements()) {
      auto base_pose_delta =
          TractorPoseDelta(base_parameters, wheel_measurement);
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
}  // namespace farm_ng
int main(int argc, char** argv) {
  // Initialize Google's logging library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // we're reading from a log, so block event_bus events from reaching the
  // calibator.
  if (FLAGS_log.empty()) {
    LOG(INFO) << "Please specify --log=file";
    return -1;
  }

  if (FLAGS_rig_calibration.empty()) {
    LOG(INFO) << "Please specify --rig_calibration=file";
    return -1;
  }

  farm_ng::SetArchivePath(FLAGS_out_archive);

  MonocularApriltagRigModel rig_model =
      farm_ng::ReadMonocularApriltagRigModelFromDisk(FLAGS_rig_calibration);

  farm_ng::EventLogReader log_reader(FLAGS_log);
  BaseToCameraModel model;
  model.set_wheel_radius(FLAGS_wheel_radius * 0.0254);
  model.set_wheel_baseline(FLAGS_wheel_baseline * 0.0254);
  farm_ng::SophusToProto(Sophus::SE3d(), "tractor/base",
                         rig_model.camera_frame_name(),
                         model.mutable_base_pose_camera());

  if (FLAGS_camera_direction == "front") {
    farm_ng::SophusToProto(
        Sophus::SE3d::rotZ(-M_PI / 2.0) * Sophus::SE3d::rotX(M_PI / 2.0) *
            Sophus::SE3d::rotY(M_PI) * Sophus::SE3d::rotZ(M_PI),
        "tractor/base", rig_model.camera_frame_name(),
        model.mutable_base_pose_camera());
  } else if (FLAGS_camera_direction == "front-upside-down") {
    farm_ng::SophusToProto(Sophus::SE3d::rotZ(-M_PI / 2.0) *
                               Sophus::SE3d::rotX(M_PI / 2.0) *
                               Sophus::SE3d::rotY(M_PI),
                           "tractor/base", rig_model.camera_frame_name(),
                           model.mutable_base_pose_camera());

  } else {
    LOG(INFO) << "Unknown --camera_direction=" << FLAGS_camera_direction;
    return -1;
  }

  BaseToCameraModel::Sample sample;

  bool has_start = false;

  // Set this to add the april tag trajectory to sample, useful for
  // visualization of pose error.
  bool full_apriltag_trajectory = true;

  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
      TractorState tractor_state;
      if (event.data().UnpackTo(&tractor_state)) {
        if (has_start) {
          farm_ng::CopyTractorStateToWheelState(
              tractor_state, sample.add_wheel_measurements());

          // TODO(ethanrublee) Tractor state is offset by some time in the log,
          // due to the latency of detecting april tags.
          // Align the tractor state to the detection.
          if (false) {
            LOG(INFO)
                << "Difference in time: "
                << google::protobuf::util::TimeUtil::DurationToMicroseconds(
                       tractor_state.stamp() -
                       sample.camera_pose_rig_start().a_pose_b().stamp()) *
                       1e-6;
          }
        }
        // LOG(INFO) << tractor_state.ShortDebugString();
      } else {
        bool calibration_sample =
            farm_ng::StartsWith(event.name(), "calibrator");
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
            farm_ng::EstimateCameraPoseRig(rig_model.rig(), detections);
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
          model.add_samples()->CopyFrom(sample);
          LOG(INFO) << "n wheel measurments: "
                    << sample.wheel_measurements().size();
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
  auto initial_resource_pb =
      farm_ng::WriteProtobufToBinaryResource("base_to_camera/initial", model);
  BaseToCameraModel solved_model;
  if (true) {
    // NOTE if you have a good initial guess of the wheel radius and baseline
    // but not the camera, it works reasonably well to first solve for just
    // base_pose_camera, with the base params held constant, then solve for both
    // jointly.
    solved_model = farm_ng::SolveBasePoseCamera(model, true);
  }
  solved_model = farm_ng::SolveBasePoseCamera(model, false);

  auto solved_resource_pb = farm_ng::WriteProtobufToBinaryResource(
      "base_to_camera/solved", solved_model);

  LOG(INFO) << "Wrote results to:\n"
            << initial_resource_pb.ShortDebugString() << "\n"
            << solved_resource_pb.ShortDebugString();
  return 0;
}
