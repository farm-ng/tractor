#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Dense>
#include <boost/asio.hpp>

#include "farm_ng/calibration/calibrate_multi_view_apriltag_rig.pb.h"
#include "farm_ng/calibration/calibrate_multi_view_lidar.pb.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"
#include "farm_ng/calibration/multi_view_lidar_model.pb.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/point_cloud.h"

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/tensor.h"
#include "farm_ng/perception/time_series.h"

#include <Eigen/Dense>
DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(output_config, "", "Output the config to a file.");
DEFINE_string(config,
              "",
              "Load config from a file rather than args.");

DEFINE_string(name, "", "Name of calibration output.");
DEFINE_string(event_log, "", "Path to event log containing input data.");
DEFINE_string(
    calibrate_multi_view_apriltag_rig_result, "",
    "Path to result of calibrate_multi_view_apriltag_rig, containing camera "
    "rig and apriltag rig to calibrate the LIDARs with respect to.");

namespace fs = boost::filesystem;

namespace farm_ng::calibration {

std::vector<std::tuple<perception::ApriltagRig::Node, std::vector<int>, double>>
ApriltagRigPointCloudMatch(const perception::ApriltagRig& rig,
                           const Eigen::Matrix3Xd& points_rig,
                           double tolerance) {
  std::vector<
      std::tuple<perception::ApriltagRig::Node, std::vector<int>, double>>
      node_indices;
  for (auto node : rig.nodes()) {
    Sophus::SE3d tag_pose_rig =
        ProtoToSophus(node.pose(), node.frame_name(), rig.name());
    Eigen::Matrix3Xd points_tag =
        perception::TransformPoints(tag_pose_rig, points_rig);
    // half the tag size + 1 pixel of border (note tag_size/8 is 1 apriltag
    // pixel)
    double half_tag = node.tag_size() / 8.0 + node.tag_size() / 2.0;

    auto [indices, error] = perception::AxisAlignedBoxFilter(
        points_tag, half_tag, half_tag, tolerance);
    if (indices.empty()) {
      continue;
    }

    node_indices.push_back({node, indices, error.z()});
  }
  return node_indices;
}

struct PointPlaneCostFunctor {
  PointPlaneCostFunctor(const Eigen::Vector3d& point_a,
                        const Eigen::Vector3d& normal_a,
                        const Eigen::Matrix3Xd& points_b,
                        const perception::SE3Map& a_pose_b_map)
      : point_a_(point_a),
        normal_a_(normal_a),
        points_b_(points_b),
        a_pose_b_map_(a_pose_b_map) {}

  template <class T>
  bool operator()(T const* const raw_a_pose_b, T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 1, Eigen::Dynamic>> residuals(raw_residuals, 1,
                                                              points_b_.cols());
    auto a_pose_b = a_pose_b_map_.Map(raw_a_pose_b);
    residuals = (normal_a_.cast<T>().transpose() *
                 (perception::TransformPoints<T>(a_pose_b, points_b_.cast<T>())
                      .colwise() -
                  point_a_.cast<T>()));
    return true;
  }
  Eigen::Vector3d point_a_;
  Eigen::Vector3d normal_a_;
  Eigen::Matrix3Xd points_b_;
  perception::SE3Map a_pose_b_map_;
};

void SavePlyFilesInTagRig(MultiViewLidarModel model, std::string prefix) {
  perception::PoseGraph posegraph;
  posegraph.AddPoses(model.camera_rig().camera_pose_rig());
  posegraph.AddPoses(model.lidar_poses());
  posegraph = posegraph.AveragePoseGraph(model.camera_rig().name());
  std::ofstream report_csv(prefix + "_report.csv");
  report_csv << "# frame_number, point_coud, tag_id, error";
  for (int i = 0; i < model.measurements_size(); ++i) {
    const auto& m_i0 = model.measurements(i);
    Sophus::SE3d camera_rig_pose_apriltag_rig =
        ProtoToSophus(m_i0.camera_rig_pose_apriltag_rig(),
                      model.camera_rig().name(), model.apriltag_rig().name());
    for (const auto& cloud :
         m_i0.multi_view_pointclouds().point_clouds_per_view()) {
      Sophus::SE3d cloud_pose_camera_rig = posegraph.CheckAverageAPoseB(
          cloud.frame_name(), model.camera_rig().name());
      Sophus::SE3d cloud_pose_apriltag_rig =
          cloud_pose_camera_rig * camera_rig_pose_apriltag_rig;
      auto points_cloud = PointCloudGetData(cloud, "xyz");
      auto points_apriltag_rig = perception::TransformPoints<double>(
          cloud_pose_apriltag_rig.inverse(), points_cloud);
      perception::SavePly(
          prefix + cloud.frame_name() + std::to_string(i) + ".ply",
          points_apriltag_rig);
      auto apriltag_rig_matches = ApriltagRigPointCloudMatch(
          model.apriltag_rig(), points_apriltag_rig, 0.1);
      for (auto [node, indices, error] : apriltag_rig_matches) {
        LOG(INFO) << node.frame_name() << "<->" << cloud.frame_name()
                  << "  matches: " << indices.size() << " error: " << error;
        report_csv << i << ", " << cloud.frame_name() << "," << node.id()
                   << ", " << error << std::endl;
        perception::SavePly(
            prefix + std::to_string(node.id()) + cloud.frame_name() +
                std::to_string(i) + ".ply",
            perception::SelectPoints(points_apriltag_rig, indices));
      }
    }
  }
}

MultiViewLidarModel Solve(MultiViewLidarModel model) {
  perception::PoseGraph posegraph;
  posegraph.AddPoses(model.camera_rig().camera_pose_rig());
  posegraph.AddPoses(model.lidar_poses());
  posegraph = posegraph.AveragePoseGraph(model.camera_rig().name());
  ceres::Problem problem;

  for (auto pose : posegraph.MutablePoseEdges()) {
    problem.AddParameterBlock(pose->GetAPoseB().data(),
                              Sophus::SE3d::num_parameters,
                              new LocalParameterizationSE3);
  }
  for (auto camera_pose : model.camera_rig().camera_pose_rig()) {
    problem.SetParameterBlockConstant(
        posegraph.MutablePoseEdge(camera_pose.frame_a(), camera_pose.frame_b())
            ->GetAPoseB()
            .data());
  }

  std::map<int, ApriltagRigTagStats> tag_stats;
  std::map<int,
           std::vector<std::tuple<PerImageRmse, ceres::ResidualBlockId, int>>>
      tag_id_to_per_image_rmse_block_id;

  for (int i = 0; i < model.measurements_size(); ++i) {
    const auto& m_i0 = model.measurements(i);
    Sophus::SE3d camera_rig_pose_apriltag_rig =
        ProtoToSophus(m_i0.camera_rig_pose_apriltag_rig(),
                      model.camera_rig().name(), model.apriltag_rig().name());
    for (const auto& cloud :
         m_i0.multi_view_pointclouds().point_clouds_per_view()) {
      auto* cloud_rig_edge = posegraph.MutablePoseEdge(
          cloud.frame_name(), model.camera_rig().name());
      Sophus::SE3d cloud_pose_camera_rig = cloud_rig_edge->GetAPoseBMapped(
          cloud.frame_name(), model.camera_rig().name());
      Sophus::SE3d cloud_pose_apriltag_rig =
          cloud_pose_camera_rig * camera_rig_pose_apriltag_rig;
      auto points_cloud = PointCloudGetData(cloud, "xyz");
      auto points_apriltag_rig = perception::TransformPoints<double>(
          cloud_pose_apriltag_rig.inverse(), points_cloud);
      auto apriltag_rig_matches = ApriltagRigPointCloudMatch(
          model.apriltag_rig(), points_apriltag_rig, 0.1);
      for (auto [node, indices, error] : apriltag_rig_matches) {
        LOG(INFO) << node.frame_name() << "<->" << cloud.frame_name()
                  << "  matches: " << indices.size() << " error: " << error;
        Sophus::SE3d apriltag_rig_pose_tag = perception::ProtoToSophus(
            node.pose(), model.apriltag_rig().name(), node.frame_name());
        Sophus::SE3d camera_rig_pose_tag =
            camera_rig_pose_apriltag_rig * apriltag_rig_pose_tag;
        // tag normal
        Eigen::Vector3d normal_camera_rig =
            camera_rig_pose_tag.rotationMatrix().col(2);
        Eigen::Vector3d point_camera_rig = camera_rig_pose_tag.translation();

        auto tag_points_cloud = perception::SelectPoints(points_cloud, indices);
        ceres::CostFunction* cost_function1 =
            new ceres::AutoDiffCostFunction<PointPlaneCostFunctor,
                                            ceres::DYNAMIC,
                                            Sophus::SE3d::num_parameters>(
                new PointPlaneCostFunctor(
                    point_camera_rig, normal_camera_rig, tag_points_cloud,
                    cloud_rig_edge->GetAPoseBMap(model.camera_rig().name(),
                                                 cloud.frame_name())),
                tag_points_cloud.cols());
        auto block_id = problem.AddResidualBlock(
            cost_function1, new ceres::CauchyLoss(0.05),
            cloud_rig_edge->GetAPoseB().data());

        ApriltagRigTagStats& stats = tag_stats[node.id()];
        stats.set_tag_id(node.id());
        stats.set_n_frames(stats.n_frames() + 1);

        PerImageRmse per_image_rmse;
        per_image_rmse.set_frame_number(i);
        per_image_rmse.set_camera_name(cloud.frame_name());

        tag_id_to_per_image_rmse_block_id[node.id()].push_back(
            std::make_tuple(per_image_rmse, block_id, tag_points_cloud.cols()));
      }
    }
  }

  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.gradient_tolerance = 1e-20;
  options.function_tolerance = 1e-20;
  options.parameter_tolerance = 1e-20;
  options.max_num_iterations = 2000;

  // Solve
  ceres::Solver::Summary summary;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = false;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport() << std::endl;

  double total_rmse = 0.0;
  double total_count = 0;
  std::vector<ApriltagRigTagStats> out_tag_stats;
  model.clear_tag_stats();
  for (auto it : tag_id_to_per_image_rmse_block_id) {
    int tag_id = it.first;
    auto& stats = tag_stats[tag_id];
    for (auto per_image_rmse_block : it.second) {
      auto [image_rmse, block, n_residuals] = per_image_rmse_block;
      double cost;
      Eigen::Matrix<double, 1, Eigen::Dynamic> residuals(1, n_residuals);
      problem.EvaluateResidualBlockAssumingParametersUnchanged(
          block, false, &cost, residuals.data(), nullptr);

      double squared_mean = residuals.squaredNorm() / residuals.cols();

      total_rmse += squared_mean;
      total_count += 1;

      stats.set_tag_rig_rmse(stats.tag_rig_rmse() + squared_mean);

      image_rmse.set_rmse(std::sqrt(squared_mean));
      stats.add_per_image_rmse()->CopyFrom(image_rmse);
    }
    stats.set_tag_rig_rmse(std::sqrt(stats.tag_rig_rmse() / stats.n_frames()));
    model.add_tag_stats()->CopyFrom(stats);
  }
  double rmse = std::sqrt(total_rmse / total_count);
  LOG(INFO) << "model rmse (meters): " << rmse << "\n";
  model.set_rmse(rmse);
  model.mutable_lidar_poses()->CopyFrom(posegraph.ToNamedSE3Poses());
  return model;
}

class CalibrateMultiViewLidarProgram {
 public:
  CalibrateMultiViewLidarProgram(
      core::EventBus& bus, CalibrateMultiViewLidarConfiguration configuration,
      bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({"^" + bus_.GetName() + "/"});
    bus_.GetEventSignal()->connect(
        std::bind(&CalibrateMultiViewLidarProgram::on_event, this,
                  std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }
    LOG(INFO) << "config:\n" << configuration_.DebugString();

    CHECK(configuration_.has_event_log()) << "Please specify an event log.";


  fs::path output_dir= fs::path(configuration_.event_log().path()).parent_path();

    // Output under the same directory as the dataset.
    core::SetArchivePath((output_dir / "multi_view_lidar_model").string());

    MultiViewLidarModel model;
    model.mutable_lidar_poses()->CopyFrom(configuration_.lidar_poses());

    auto mv_rig_result = core::ReadProtobufFromResource<
        calibration::CalibrateMultiViewApriltagRigResult>(
        configuration_.calibrate_multi_view_apriltag_rig_result());

    LOG(INFO) << mv_rig_result.DebugString();
    model.mutable_camera_rig()->CopyFrom(
        core::ReadProtobufFromResource<perception::MultiViewCameraRig>(
            mv_rig_result.camera_rig_solved()));
    const auto& camera_rig = model.camera_rig();
    model.mutable_apriltag_rig()->CopyFrom(
        core::ReadProtobufFromResource<perception::ApriltagRig>(
            mv_rig_result.apriltag_rig_solved()));
    const auto& apriltag_rig = model.apriltag_rig();
    std::map<std::string, perception::TimeSeries<core::Event>> event_series;

    std::set<std::string> lidar_names;
    if (configuration_.include_lidars_size() > 0) {
      for (auto lidar_name : configuration_.include_lidars()) {
        lidar_names.insert(lidar_name);
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

      if (event.data().Is<perception::PointCloud>()) {
        if (configuration_.include_lidars_size() == 0) {
          lidar_names.insert(event.name());
        }
      }
      event_series[event.name()].insert(event);
    }
    CHECK_GT(lidar_names.size(), 0);

    auto time_window =
        google::protobuf::util::TimeUtil::MillisecondsToDuration(200);

    perception::ApriltagsFilter tag_filter;
    std::string root_camera_name = camera_rig.root_camera_name();

    int steady_count = 2;

    for (auto event : event_series[root_camera_name + "/apritags"]) {
      perception::ApriltagDetections detections;
      CHECK(event.data().UnpackTo(&detections));
      if (!tag_filter.AddApriltags(detections, steady_count, 7)) {
        continue;
      }

      MultiViewLidarModel::Measurement measurement;
      for (auto lidar_name : lidar_names) {
        auto closest_event =
            event_series[lidar_name].FindNearest(event.stamp(), time_window);

        CHECK(closest_event && closest_event->stamp() == event.stamp())
            << closest_event->name() << " "
            << closest_event->stamp().ShortDebugString()
            << " != " << event.name() << " "
            << event.stamp().ShortDebugString();
        auto* cloud = measurement.mutable_multi_view_pointclouds()
                          ->add_point_clouds_per_view();
        CHECK(closest_event->data().UnpackTo(cloud))
            << closest_event->name() << " is not a PointCloud.";
      }
      auto* mv_detections = measurement.mutable_multi_view_detections();
      for (auto camera : camera_rig.cameras()) {
        // HACK
        std::string apriltags_topic = camera.frame_name() + "/apritags";
        CHECK(event_series.count(apriltags_topic) > 0)
            << apriltags_topic << " not in event log";
        auto closest_event = event_series[apriltags_topic].FindNearest(
            event.stamp(), time_window);
        // HACK we only have 3 images in current dataset per point cloud.
        CHECK(closest_event && closest_event->stamp() == event.stamp())
            << closest_event->name() << " "
            << closest_event->stamp().ShortDebugString()
            << " != " << event.name() << " "
            << event.stamp().ShortDebugString();
        CHECK(closest_event->data().UnpackTo(
            mv_detections->add_detections_per_view()))
            << closest_event->name() << " is not an ApriltagDetections.";
      }
      // std::optional<std::tuple<perception::NamedSE3Pose, double,
      //                         std::vector<ApriltagRigTagStats>>>
      auto camera_rig_pose_est = EstimateMultiViewCameraRigPoseApriltagRig(
          camera_rig, apriltag_rig, *mv_detections);
      if (!camera_rig_pose_est) {
        LOG(WARNING) << "Couldn't estimate camera pose for frame: "
                     << event.name() << " " << event.stamp().ShortDebugString();
        continue;
      }
      auto [pose, rmse, stats] = *camera_rig_pose_est;
      measurement.mutable_camera_rig_pose_apriltag_rig()->CopyFrom(pose);
      model.add_measurements()->CopyFrom(measurement);
    }
    SavePlyFilesInTagRig(model, "/blobstore/scratch/init_");
    model = Solve(model);
    SavePlyFilesInTagRig(model, "/blobstore/scratch/solved_");

    auto temp_model= model;
    temp_model.clear_measurements();

    auto model_json = core::GetUniqueArchiveResource("multi_view_lidar_model", "json", core::ContentTypeProtobufJson<MultiViewLidarModel>());
    LOG(INFO) << "writing: " << model_json.second.string();
    core::WriteProtobufToJsonFile(model_json.second, temp_model);

    LOG(INFO) << "Measurements: " << model.measurements_size();
    CalibrateMultiViewLidarResult result;

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
    timer_.async_wait(std::bind(&CalibrateMultiViewLidarProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const core::Event& event) {
    CalibrateMultiViewLidarConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CalibrateMultiViewLidarConfiguration configuration) {
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
  CalibrateMultiViewLidarConfiguration configuration_;
  CalibrateMultiViewLidarStatus status_;
  CalibrateMultiViewLidarResult result_;
};

}  // namespace farm_ng::calibration

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::CalibrateMultiViewLidarConfiguration config;
  if (!FLAGS_config.empty()) {
    config = farm_ng::core::ReadProtobufFromJsonFile<
        farm_ng::calibration::CalibrateMultiViewLidarConfiguration>(
        FLAGS_config);
    farm_ng::calibration::CalibrateMultiViewLidarProgram program(
        bus, config, FLAGS_interactive);
    return program.run();
  } else {
    config.set_name(FLAGS_name);
    config.mutable_event_log()->CopyFrom(
        farm_ng::core::EventLogResource(FLAGS_event_log));
    config.mutable_calibrate_multi_view_apriltag_rig_result()->CopyFrom(
        farm_ng::core::ProtobufJsonResource<
            farm_ng::calibration::CalibrateMultiViewApriltagRigResult>(
            FLAGS_calibrate_multi_view_apriltag_rig_result));
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
