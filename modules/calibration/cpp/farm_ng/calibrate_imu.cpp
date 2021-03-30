//
// ./build/modules/calibration/cpp/farm_ng/calibrate_imu
// --event_log /blobstore/logs/calibration_capture/events.log
// --name "imu01"
// --calibrate_multi_view_apriltag_rig_result=logs/calibration_capture/multi_view_apriltag_rig_model/flir_rig.json
// --align_sensor_rig_result
// /blobstore/logs/calibration_capture/aligned_sensor_rig.json

#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Dense>
#include <boost/asio.hpp>

#include "farm_ng/calibration/align_sensor_rig.pb.h"
#include "farm_ng/calibration/calibrate_imu.pb.h"
#include "farm_ng/calibration/calibrate_multi_view_apriltag_rig.pb.h"

#include "farm_ng/calibration/imu_model.pb.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/imu.pb.h"

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include <opencv2/imgproc.hpp>
#include "farm_ng/perception/tensor.h"
#include "farm_ng/perception/time_series.h"

#include <Eigen/Dense>
DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(output_config, "", "Output the config to a file.");
DEFINE_string(config, "", "Load config from a file rather than args.");

DEFINE_string(name, "", "Name of calibration output.");
DEFINE_string(event_log, "", "Path to event log containing input data.");
DEFINE_string(align_sensor_rig_result, "",
              "Path to aligned sensor rig output.");
DEFINE_string(
    calibrate_multi_view_apriltag_rig_result, "",
    "Path to result of calibrate_multi_view_apriltag_rig, containing camera "
    "rig and apriltag rig to calibrate the IMU with respect to.");

namespace fs = boost::filesystem;

namespace farm_ng::calibration {
struct SinglePoseCostFunction {
  SinglePoseCostFunction(Sophus::SE3d a0_pose_a1, Sophus::SE3d b0_pose_b1)
      : a0_pose_a1_(a0_pose_a1), b0_pose_b1_(b0_pose_b1) {}

  template <class T>
  bool operator()(T const* const raw_a_pose_b, T* raw_residuals) const {
    // Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_0(raw_residuals);
    // Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_1(raw_residuals + 3);
    // Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_2(raw_residuals + 6);
    // Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_3(raw_residuals + 9);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);

    Eigen::Map<Sophus::SE3<T> const> a_pose_b(raw_a_pose_b);
    Sophus::SE3<T> a_pose_a = a0_pose_a1_.cast<T>() * a_pose_b *
                              b0_pose_b1_.inverse().cast<T>() *
                              a_pose_b.inverse();

    residuals = a_pose_a.log();

    // typedef Eigen::Matrix<T, 3, 1> Point3;
    // Point3 orig(T(0.0), T(0.0), T(0.0));
    // Point3 x(T(1.0), T(0.0), T(0.0));
    // Point3 y(T(0.0), T(1.0), T(0.0));
    // Point3 z(T(0.0), T(0.0), T(1.0));
    // residuals_0 = z - (a_pose_a * z);
    // residuals_1 = x - (a_pose_a * x);
    // residuals_2 = y - (a_pose_a * y);
    // residuals_3 = a_pose_a * orig;
    return true;
  }

  Sophus::SE3d a0_pose_a1_;
  Sophus::SE3d b0_pose_b1_;
};

ImuModel Solve(ImuModel imu_model) {
  perception::PoseGraph pose_graph;
  pose_graph.AddPoses(imu_model.base_pose_sensor());
  pose_graph = pose_graph.AveragePoseGraph(imu_model.base_frame());
  auto base_pose_camera = pose_graph.CheckAverageNamedSE3Pose(
      imu_model.base_frame(), imu_model.camera_rig().root_camera_name());
  perception::NamedSE3Pose base_pose_rig;
  {
    perception::PoseGraph crig_pose_graph;
    crig_pose_graph.AddPoses(imu_model.camera_rig().camera_pose_rig());
    crig_pose_graph.AveragePoseGraph(imu_model.camera_rig().root_camera_name());
    auto camera_pose_rig = crig_pose_graph.CheckAverageNamedSE3Pose(
        imu_model.camera_rig().root_camera_name(),
        imu_model.camera_rig().name());
    base_pose_rig = Multiply(base_pose_camera, camera_pose_rig);
  }

  Sophus::SE3d base_pose_imu =
      Sophus::SE3d::trans(0.69495304, -0.41572446, 1.1779071) *
      Sophus::SE3d::rotX(0.00102145) * Sophus::SE3d::rotY(0.00102145) *
      Sophus::SE3d::rotZ(-5.2168e-07) * Sophus::SE3d::rotZ(-M_PI / 2);

  ceres::Problem problem;

  AddSE3ParameterBlockSubsetTranslation(&problem, base_pose_imu.data(),
                                        std::vector<int>({2}));

  // problem.AddParameterBlock(base_pose_imu.data(),
  // Sophus::SE3d::num_parameters, new LocalParameterizationSE3);
  // problem.SetParameterBlockConstant(base_pose_imu.data());

  for (int i = 0; i < imu_model.measurements_size() - 1; ++i) {
    for (int j = i + 1; j < imu_model.measurements_size(); ++j) {
      auto m_i0 = imu_model.measurements(i);
      auto m_i1 = imu_model.measurements(j);
      CHECK_EQ(m_i0.camera_rig_pose_apriltag_rig().frame_a(),
               imu_model.camera_rig().name());
      CHECK_EQ(m_i1.camera_rig_pose_apriltag_rig().frame_a(),
               imu_model.camera_rig().name());
      auto base_i0_pose_base_i1 =
          Multiply(Multiply(base_pose_rig, m_i0.camera_rig_pose_apriltag_rig()),
                   Inverse(Multiply(base_pose_rig,
                                    m_i1.camera_rig_pose_apriltag_rig())));

      auto init_pose_imu_i0 = m_i0.imu().orientation();
      CHECK_EQ(init_pose_imu_i0.frame_b(), imu_model.imu_name());
      auto init_pose_imu_i1 = m_i1.imu().orientation();
      CHECK_EQ(init_pose_imu_i1.frame_b(), imu_model.imu_name());
      auto imu_i0_pose_imu_i1 =
          Multiply(Inverse(init_pose_imu_i0), init_pose_imu_i1);

      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<SinglePoseCostFunction, 6,
                                          Sophus::SE3d::num_parameters>(
              new SinglePoseCostFunction(
                  ProtoToSophus(base_i0_pose_base_i1.a_pose_b()),
                  ProtoToSophus(imu_i0_pose_imu_i1.a_pose_b())));

      problem.AddResidualBlock(cost_function, nullptr, base_pose_imu.data());
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.gradient_tolerance = 1e-18;
  options.function_tolerance = 1e-18;
  options.parameter_tolerance = 1e-18;
  options.max_num_iterations = 2000;

  // Solve
  ceres::Solver::Summary summary;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = true;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport() << std::endl;
  perception::NamedSE3Pose base_pose_imu_proto;
  perception::SophusToProto(base_pose_imu, imu_model.base_frame(),
                            imu_model.imu_name(), &base_pose_imu_proto);
  imu_model.add_base_pose_sensor()->CopyFrom(base_pose_imu_proto);
  LOG(INFO) << base_pose_imu_proto.ShortDebugString();

  return imu_model;
}

class CalibrateImuProgram {
 public:
  CalibrateImuProgram(core::EventBus& bus,
                      CalibrateImuConfiguration configuration, bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({"^" + bus_.GetName() + "/"});
    bus_.GetEventSignal()->connect(
        std::bind(&CalibrateImuProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }
    LOG(INFO) << "config:\n" << configuration_.DebugString();

    CHECK(configuration_.has_event_log()) << "Please specify an event log.";

    fs::path output_dir =
        fs::path(configuration_.event_log().path()).parent_path();

    // Output under the same directory as the dataset.
    core::SetArchivePath((output_dir / "imu_model").string());

    ImuModel imu_model;

    auto mv_rig_result = core::ReadProtobufFromResource<
        calibration::CalibrateMultiViewApriltagRigResult>(
        configuration_.calibrate_multi_view_apriltag_rig_result());

    LOG(INFO) << mv_rig_result.DebugString();

    imu_model.mutable_camera_rig()->CopyFrom(
        core::ReadProtobufFromResource<perception::MultiViewCameraRig>(
            mv_rig_result.camera_rig_solved()));
    const auto& camera_rig = imu_model.camera_rig();

    imu_model.mutable_apriltag_rig()->CopyFrom(
        core::ReadProtobufFromResource<perception::ApriltagRig>(
            mv_rig_result.apriltag_rig_solved()));

    if (configuration_.has_align_sensor_rig_result()) {
      auto aligned_sensor_rig =
          core::ReadProtobufFromResource<AlignSensorRigResult>(
              configuration_.align_sensor_rig_result());
      imu_model.set_base_frame(aligned_sensor_rig.base_frame());
      imu_model.mutable_base_pose_sensor()->CopyFrom(
          aligned_sensor_rig.base_pose_sensor());
    } else {
      imu_model.set_base_frame(imu_model.camera_rig().name());
      imu_model.mutable_base_pose_sensor()->CopyFrom(
          imu_model.camera_rig().camera_pose_rig());
    }

    const auto& apriltag_rig = imu_model.apriltag_rig();

    std::map<std::string, perception::TimeSeries<core::Event>> event_series;

    std::set<std::string> imu_names;
    if (configuration_.include_imus_size() > 0) {
      for (auto imu_name : configuration_.include_imus()) {
        imu_names.insert(imu_name);
      }
    }
    core::EventLogReader log_reader(configuration_.event_log());
    while (true) {
      core::Event event;
      try {
        event = log_reader.ReadNext();
      } catch (const std::runtime_error& e) {
        break;
      }

      if (event.data().Is<perception::Imu>()) {
        if (configuration_.include_imus_size() == 0) {
          imu_names.insert(event.name());
        }
      }
      event_series[event.name()].insert(event);
    }
    CHECK_EQ(imu_names.size(), 1);
    imu_model.set_imu_name(*imu_names.begin());

    auto time_window =
        google::protobuf::util::TimeUtil::MillisecondsToDuration(1000);

    perception::ApriltagsFilter tag_filter;
    std::string root_camera_name = camera_rig.root_camera_name();

    int steady_count = 2;
    for (auto event : event_series[root_camera_name + "/apriltags"]) {
      perception::ApriltagDetections detections;
      CHECK(event.data().UnpackTo(&detections));
      if (!tag_filter.AddApriltags(detections, steady_count, 7)) {
        continue;
      }
      ImuModel::Measurement measurement;
      for (auto imu_name : imu_names) {
        auto closest_event =
            event_series[imu_name].FindNearest(event.stamp(), time_window);
        if (!closest_event) {
          LOG(WARNING) << "No closest event for: " << imu_name
                       << " within time_window: "
                       << time_window.ShortDebugString();
          continue;
        }

        CHECK(closest_event->data().UnpackTo(measurement.mutable_imu()))
            << closest_event->name() << " is not an Imu";
        CHECK_EQ(measurement.imu().orientation().frame_b(), imu_name);
      }

      perception::MultiViewApriltagDetections mv_detections;

      for (auto camera : camera_rig.cameras()) {
        std::string apriltags_topic = camera.frame_name() + "/apriltags";
        auto closest_event = event_series[apriltags_topic].FindNearest(
            event.stamp(), time_window);
        CHECK(closest_event->data().UnpackTo(
            mv_detections.add_detections_per_view()))
            << closest_event->name() << " is not an ApriltagDetections.";
      }
      // std::optional<std::tuple<perception::NamedSE3Pose, double,
      //                         std::vector<ApriltagRigTagStats>>>
      auto camera_rig_pose_est = EstimateMultiViewCameraRigPoseApriltagRig(
          camera_rig, apriltag_rig, mv_detections);
      if (!camera_rig_pose_est) {
        LOG(WARNING) << "Couldn't estimate camera pose for frame: "
                     << event.name() << " " << event.stamp().ShortDebugString();
        continue;
      }
      auto [pose, rmse, stats] = *camera_rig_pose_est;
      measurement.mutable_camera_rig_pose_apriltag_rig()->CopyFrom(pose);
      imu_model.add_measurements()->CopyFrom(measurement);
    }
    imu_model = Solve(imu_model);
    send_status();
    return 0;
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
    timer_.async_wait(
        std::bind(&CalibrateImuProgram::on_timer, this, std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const core::Event& event) {
    CalibrateImuConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CalibrateImuConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const core::Event& event) {
    CHECK(event.name().rfind(bus_.GetName() + "/", 0) == 0);
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CalibrateImuConfiguration configuration_;
  CalibrateImuStatus status_;
  CalibrateImuResult result_;
};

}  // namespace farm_ng::calibration

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::CalibrateImuConfiguration config;
  if (!FLAGS_config.empty()) {
    config = farm_ng::core::ReadProtobufFromJsonFile<
        farm_ng::calibration::CalibrateImuConfiguration>(FLAGS_config);
    farm_ng::calibration::CalibrateImuProgram program(bus, config,
                                                      FLAGS_interactive);
    return program.run();
  } else {
    config.set_name(FLAGS_name);
    config.mutable_event_log()->CopyFrom(
        farm_ng::core::EventLogResource(FLAGS_event_log));
    config.mutable_calibrate_multi_view_apriltag_rig_result()->CopyFrom(
        farm_ng::core::ProtobufJsonResource<
            farm_ng::calibration::CalibrateMultiViewApriltagRigResult>(
            FLAGS_calibrate_multi_view_apriltag_rig_result));

    if (!FLAGS_align_sensor_rig_result.empty()) {
      config.mutable_align_sensor_rig_result()->CopyFrom(
          farm_ng::core::ProtobufJsonResource<
              farm_ng::calibration::AlignSensorRigResult>(
              FLAGS_align_sensor_rig_result));
    }
    farm_ng::calibration::CalibrateImuProgram program(bus, config,
                                                      FLAGS_interactive);
    return program.run();
  }
  if (!FLAGS_output_config.empty()) {
    farm_ng::core::WriteProtobufToJsonFile(FLAGS_output_config, config);
    return 0;
  }
  LOG(ERROR) << "Please provide a config.";
  return -1;
}
int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
