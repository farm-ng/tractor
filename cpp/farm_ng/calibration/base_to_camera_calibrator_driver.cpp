#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ceres/ceres.h>
#include <google/protobuf/util/json_util.h>

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/local_parameterization_se3.h"
#include "farm_ng/calibration/pose_utils.h"

#include "farm_ng/event_log_reader.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/tractor.pb.h"

DEFINE_string(log, "", "Path to log file, recorded with farm-ng-ipc-logger");
DEFINE_string(
    rig_calibration, "",
    "Path to a rig calibration file, recorded with farm-ng-ipc-logger");

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
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

struct BaseToCameraSample {
  Sophus::SE3d camera_pose_rig_start;
  Sophus::SE3d camera_pose_rig_end;
  // (left,right, dt),... radians per second
  std::vector<Eigen::Vector3d> wheel_velocities;
};

template <typename T>
static Sophus::SE3<T> TractorPoseDelta(
    const Eigen::Matrix<T, 2, 1>& base_parameters,
    const Eigen::Matrix<T, 3, 1>& wheel_velocity) {
  const T& wheel_radius = base_parameters[0];
  const T& wheel_baseline = base_parameters[1];
  const T& vel_left = wheel_velocity[0];
  const T& vel_right = wheel_velocity[1];
  const T& dt = wheel_velocity[2];
  T v = T(wheel_radius / 2.0) * (vel_left + vel_right);
  T w = (wheel_radius / wheel_baseline) * (vel_right - vel_left);
  Eigen::Matrix<T, 6, 1> x;
  x << v * dt, T(0), T(0), T(0), T(0), w * dt;
  return Sophus::SE3<T>::exp(x);
}
template <typename T>
Sophus::SE3<T> TractorStartPoseTractorEnd(
    const Eigen::Matrix<T, 2, 1>& base_params,
    const BaseToCameraSample& sample) {
  Sophus::SE3<T> tractor_start_pose_tractor_end =
      Sophus::SE3d::rotZ(0).cast<T>();
  for (const auto& wheel_velocity : sample.wheel_velocities) {
    tractor_start_pose_tractor_end =
        tractor_start_pose_tractor_end *
        TractorPoseDelta<T>(base_params, wheel_velocity.cast<T>());
  }
  return tractor_start_pose_tractor_end;
}

struct BaseModel {
  Sophus::SE3d base_pose_camera;
  Eigen::Vector2d base_parameters;
};

struct BasePoseCameraCostFunctor {
  BasePoseCameraCostFunctor(const BaseToCameraSample& sample)
      : sample_(sample) {}

  template <class T>
  bool operator()(T const* const raw_base_pose_camera,
                  T const* const raw_base_params, T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const base_pose_camera(
        raw_base_pose_camera);
    Eigen::Map<Eigen::Matrix<T, 2, 1> const> const base_params(raw_base_params);
    const Sophus::SE3<T> camera_end_pose_camera_start(
        (sample_.camera_pose_rig_end * sample_.camera_pose_rig_start.inverse())
            .cast<T>());
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);

    Sophus::SE3<T> tractor_start_pose_tractor_end =
        TractorStartPoseTractorEnd<T>(base_params, sample_);

    auto tractor_start_pose_tractor_start =
        tractor_start_pose_tractor_end * base_pose_camera *
        camera_end_pose_camera_start * base_pose_camera.inverse();

    residuals = tractor_start_pose_tractor_start.log();
    return true;
  }
  BaseToCameraSample sample_;
};

class BaseToCameraIterationCallback : public ceres::IterationCallback {
 public:
  explicit BaseToCameraIterationCallback(const BaseModel* model)
      : model_(model) {}

  ~BaseToCameraIterationCallback() {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    LOG(INFO) << "trans: " << model_->base_pose_camera.translation().transpose()
              << " rot:"
              << model_->base_pose_camera.unit_quaternion().coeffs().transpose()
              << " base parameters: "
              << model_->base_parameters.transpose() / 0.0254;
    return ceres::SOLVER_CONTINUE;
  }

 private:
  const BaseModel* model_;
};

Sophus::optional<BaseModel> SolveBasePoseCamera(
    BaseModel base_model, std::vector<BaseToCameraSample>& samples) {
  ceres::Problem problem;

  problem.AddParameterBlock(base_model.base_pose_camera.data(),
                            SE3d::num_parameters, new LocalParameterizationSE3);

  problem.AddParameterBlock(base_model.base_parameters.data(), 2, nullptr);
  problem.SetParameterBlockConstant(base_model.base_parameters.data());

  for (auto sample : samples) {
    ceres::CostFunction* cost_function1 =
        new ceres::AutoDiffCostFunction<BasePoseCameraCostFunctor, 6,
                                        Sophus::SE3d::num_parameters, 2>(
            new BasePoseCameraCostFunctor(sample));
    problem.AddResidualBlock(cost_function1, new ceres::HuberLoss(1.0),
                             base_model.base_pose_camera.data(),
                             base_model.base_parameters.data());
  }

  BaseToCameraIterationCallback callback(&base_model);
  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.gradient_tolerance = 1e-6;
  options.function_tolerance = 1e-6;
  options.parameter_tolerance = 1e-6;
  options.max_num_iterations = 2000;
  options.callbacks.push_back(&callback);

  // Solve
  ceres::Solver::Summary summary;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport() << std::endl;

  LOG(INFO) << "root mean of residual error: "
            << std::sqrt(summary.final_cost / summary.num_residuals);
  LOG(INFO) << "trans: "
            << base_model.base_pose_camera.translation().transpose()
            << " rot:\n"
            << base_model.base_pose_camera.unit_quaternion().matrix()
            << " \n base parameters: "
            << base_model.base_parameters.transpose() / 0.0254;
  return base_model;

}  // namespace farm_ng

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

  MonocularApriltagRigModel model =
      farm_ng::ReadMonocularApriltagRigModelFromDisk(FLAGS_rig_calibration);

  farm_ng::EventLogReader log_reader(FLAGS_log);

  farm_ng::BaseModel base_model;
  base_model.base_parameters[0] = (17.0 / 2.0) * 0.0254;
  base_model.base_parameters[1] = (42) * 0.0254;

  base_model.base_pose_camera = Sophus::SE3d::rotZ(
      0);  // n-M_PI / 2.0) *
           //                              Sophus::SE3d::rotX(M_PI / 2.0) *
           // Sophus::SE3d::rotY(M_PI);

  base_model.base_pose_camera.translation().x() = 0.0;
  base_model.base_pose_camera.translation().y() = 0.0;
  base_model.base_pose_camera.translation().z() = 1.0;

  farm_ng::BaseToCameraSample sample;
  bool has_start = false;

  std::vector<farm_ng::BaseToCameraSample> samples;

  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
      TractorState tractor_state;
      if (event.data().UnpackTo(&tractor_state)) {
        if (has_start) {
          Eigen::Vector3d wheel_velocity(
              tractor_state.wheel_velocity_rads_right(),
              tractor_state.wheel_velocity_rads_left(), tractor_state.dt());

          sample.wheel_velocities.push_back(wheel_velocity);
        }
        // LOG(INFO) << tractor_state.ShortDebugString();
      } else if (farm_ng::StartsWith(event.name(), "calibrator")) {
        ApriltagDetections detections;
        if (!event.data().UnpackTo(&detections)) {
          continue;
        }
        if (detections.detections_size() == 0) {
          continue;
        }
        // LOG(INFO) << event.ShortDebugString();
        Sophus::optional<Sophus::SE3d> o_camera_pose_rig =
            farm_ng::EstimateCameraPoseRig(model.rig(), detections);
        if (o_camera_pose_rig) {
          if (!has_start) {
            sample.camera_pose_rig_start = *o_camera_pose_rig;
            has_start = true;
          } else {
            sample.camera_pose_rig_end = *o_camera_pose_rig;
            samples.push_back(sample);
            LOG(INFO) << "n wheel velocities: "
                      << sample.wheel_velocities.size()
                      << " camera_start_pose_camera_end: "
                      << (sample.camera_pose_rig_start *
                          sample.camera_pose_rig_end.inverse())
                             .translation()
                             .transpose();
            sample.camera_pose_rig_start = *o_camera_pose_rig;
            sample.wheel_velocities.clear();
          }
        }
      }

    } catch (std::runtime_error& e) {
      break;
    }
  }

  farm_ng::SolveBasePoseCamera(base_model, samples);

  return 0;
}
