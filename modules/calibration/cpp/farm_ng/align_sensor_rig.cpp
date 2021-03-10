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

#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/tensor.h"

#include "farm_ng/perception/time_series.h"

#include <Eigen/Dense>

DEFINE_string(result, "aligned_sensor_rig.json", "Output path.");
DEFINE_string(output_config, "",
              "output a config json file to the given path.");
DEFINE_string(config, "", "Load config from ajson file.");

DEFINE_string(multi_view_lidar_model, "",
              "Path to the multi_view_lidar_model to align");

DEFINE_int32(floor_tag, 313, "A tag on the floor.");
DEFINE_double(floor_distance_threshold, 0.1,
              "Distance from floor to consider tags belonging to the floor.");

namespace fs = boost::filesystem;

namespace farm_ng::calibration {
Eigen::Matrix3Xd ToMatrix(const std::vector<Eigen::Vector3d>& points) {
  return Eigen::Map<const Eigen::Matrix3Xd>(points[0].data(), 3, points.size());
}
Eigen::Hyperplane<double, 3> FitPlaneToPoints(const Eigen::Matrix3Xd& points) {
  Eigen::Vector3d mean = points.rowwise().mean();
  Eigen::Matrix3Xd points_centered = points.colwise() - mean;
  auto svd =
      points_centered.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeThinV);
  Eigen::Vector3d normal = svd.matrixU().col(2);
  LOG(INFO) << "p=" << mean.transpose() << " n=" << normal.transpose();
  return Eigen::Hyperplane<double, 3>(normal, mean);
}

// based on https://github.com/nghiaho12/rigid_transform_3D/blob/master/rigid_transform_3D.py
// http://nghiaho.com/?page_id=671
Sophus::SE3d FitAPoseB(const Eigen::Matrix3Xd& points_a,
                       const Eigen::Matrix3Xd& points_b) {
  Eigen::Vector3d mean_a = points_a.rowwise().mean();
  Eigen::Vector3d mean_b = points_b.rowwise().mean();
  Eigen::Matrix3Xd points_a_centered = points_a.colwise() - mean_a;
  Eigen::Matrix3Xd points_b_centered = points_b.colwise() - mean_b;
  Eigen::MatrixXd a_H_b = points_a_centered * points_b_centered.transpose();
  auto svd = a_H_b.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::Matrix3d b_R_a = V * U.transpose();
  if (b_R_a.determinant() < 0) {
    V.col(2) *= -1;
    b_R_a = V * U.transpose();
    CHECK_GE(b_R_a.determinant(), 0);
  }
  Eigen::Matrix4d b_pose_a = Eigen::Matrix4d::Identity();
  b_pose_a.block<3, 3>(0, 0) = b_R_a;
  b_pose_a.block<3, 1>(0, 3) = mean_b - b_R_a * mean_a;
  auto a_pose_b = Sophus::SE3d(b_pose_a).inverse();
  return a_pose_b;
}

int align_sensor_rig(AlignSensorRigConfiguration config) {
  CHECK_GT(config.floor_tag_ids_size(), 0);
  MultiViewLidarModel lidar_model =
      core::ReadProtobufFromResource<MultiViewLidarModel>(config.model());

  perception::PoseGraph pose_graph;
  for (const perception::ApriltagRig::Node& node :
       lidar_model.apriltag_rig().nodes()) {
    pose_graph.AddPose(node.pose());
  }
  std::string floor_tag_name = perception::FrameRigTag(
      lidar_model.apriltag_rig().name(), config.floor_tag_ids(0));
  perception::PoseGraph floor_tag_graph =
      pose_graph.AveragePoseGraph(floor_tag_name);
  double floor_distance_thresh = config.floor_distance_threshold();
  std::vector<Eigen::Vector3d> floor_points_tag_rig;

  for (const perception::ApriltagRig::Node& node :
       lidar_model.apriltag_rig().nodes()) {
    auto floor_tag_pose_tag =
        floor_tag_graph.CheckAverageAPoseB(floor_tag_name, node.frame_name());
    double err = 0;
    auto points_tag = perception::PointsTag(node);
    for (auto point_tag : points_tag) {
      // LOG(INFO) << "point_tag: " << point_tag.transpose();
      err = std::max(err, std::abs((floor_tag_pose_tag * point_tag).z()));
    }
    if (err < floor_distance_thresh) {
      LOG(INFO) << node.frame_name() << " on floor err: " << err;
      Sophus::SE3d tag_rig_pose_tag = ProtoToSophus(
          node.pose(), lidar_model.apriltag_rig().name(), node.frame_name());
      for (auto point_tag : points_tag) {
        floor_points_tag_rig.push_back(tag_rig_pose_tag * point_tag);
      }
    } else {
      // LOG(INFO) << node.frame_name() << " err : " << err;
    }
  }

  std::vector<Eigen::Vector3d> points_camera_rig;

  for (int i = 0; i < lidar_model.measurements_size(); ++i) {
    const auto& m_i0 = lidar_model.measurements(i);
    Sophus::SE3d camera_rig_pose_apriltag_rig = ProtoToSophus(
        m_i0.camera_rig_pose_apriltag_rig(), lidar_model.camera_rig().name(),
        lidar_model.apriltag_rig().name());
    for (auto floor_point_tag_rig : floor_points_tag_rig) {
      points_camera_rig.push_back(camera_rig_pose_apriltag_rig *
                                  floor_point_tag_rig);
    }
  }
  perception::PoseGraph sensor_rig_graph;
  sensor_rig_graph.AddPoses(lidar_model.lidar_poses());
  auto plane_camera_rig = FitPlaneToPoints(ToMatrix(points_camera_rig));
  for (auto pose : lidar_model.lidar_poses()) {
    auto sensor_name = pose.frame_b();
    auto camera_rig_pose_sensor = sensor_rig_graph.CheckAverageAPoseB(
        lidar_model.camera_rig().name(), sensor_name);
    LOG(INFO) << "sensor_name: " << sensor_name << " distance to ground: "
              << plane_camera_rig.signedDistance(
                     camera_rig_pose_sensor.translation());
  }

  std::vector<Eigen::Vector3d> points_base;
  std::vector<Eigen::Vector3d> points_rig;
  Eigen::Hyperplane<double, 3> plane_base(Eigen::Vector3d(0, 0, 1.0),
                                          Eigen::Vector3d(0, 0, 0));
  for (auto pose : config.base_pose_sensor_measured()) {
    Sophus::SE3d base_pose_sensor =
        perception::ProtoToSophus(pose, config.base_frame(), pose.frame_b());
    points_base.push_back(base_pose_sensor.translation());
    points_base.push_back(
        plane_base.projection(base_pose_sensor.translation()));

    Sophus::SE3d rig_pose_sensor = sensor_rig_graph.CheckAverageAPoseB(
        lidar_model.camera_rig().name(), pose.frame_b());
    points_rig.push_back(rig_pose_sensor.translation());
    points_rig.push_back(
        plane_camera_rig.projection(rig_pose_sensor.translation()));
  }
  Sophus::SE3d base_pose_rig =
      FitAPoseB(ToMatrix(points_base), ToMatrix(points_rig));
  AlignSensorRigResult result;
  result.set_base_frame(config.base_frame());
  for (auto pose : lidar_model.lidar_poses()) {
    auto sensor_name = pose.frame_b();
    auto camera_rig_pose_sensor = sensor_rig_graph.CheckAverageAPoseB(
        lidar_model.camera_rig().name(), sensor_name);
    auto base_pose_sensor = base_pose_rig * camera_rig_pose_sensor;
    perception::SophusToProto(base_pose_sensor, config.base_frame(),
                              sensor_name, result.add_base_pose_sensor());
    LOG(INFO) << result.base_pose_sensor(result.base_pose_sensor_size() - 1)
                     .ShortDebugString();
  }

  farm_ng::core::WriteProtobufToJsonFile(FLAGS_result, result);

  return 0;
}

}  // namespace farm_ng::calibration

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;
  std::string filename = boost::filesystem::path(argv[0]).filename().string();
  google::InitGoogleLogging(filename.c_str());
  farm_ng::calibration::AlignSensorRigConfiguration config;
  if (!FLAGS_config.empty()) {
    config = farm_ng::core::ReadProtobufFromJsonFile<
        farm_ng::calibration::AlignSensorRigConfiguration>(FLAGS_config);
  } else {
    config.set_floor_distance_threshold(FLAGS_floor_distance_threshold);
    config.add_floor_tag_ids(FLAGS_floor_tag);
    config.mutable_model()->set_path(FLAGS_multi_view_lidar_model);
    config.mutable_model()->set_content_type(
        farm_ng::core::ContentTypeProtobufBinary<
            farm_ng::calibration::MultiViewLidarModel>());
  }
  if (!FLAGS_output_config.empty()) {
    farm_ng::core::WriteProtobufToJsonFile(FLAGS_output_config, config);
    return 0;
  }
  return farm_ng::calibration::align_sensor_rig(config);
}
