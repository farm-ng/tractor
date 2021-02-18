#ifndef FARM_NG_PERCEPTION_POINT_CLOUD_H_
#define FARM_NG_PERCEPTION_POINT_CLOUD_H_
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <farm_ng/perception/point_cloud.pb.h>

namespace farm_ng::perception {

template <typename Scalar>
Eigen::Matrix<Scalar, 3, Eigen::Dynamic> TransformPoints(
    const Sophus::SE3<Scalar>& a_pose_b,
    const Eigen::Matrix<Scalar, 3, Eigen::Dynamic>& xyz_b) {
  Eigen::Matrix<Scalar, 4, Eigen::Dynamic> h_xyz_b =
      Eigen::Matrix<Scalar, 4, Eigen::Dynamic>::Ones(4, xyz_b.cols());
  h_xyz_b.topRows(3) = xyz_b;
  return a_pose_b.matrix3x4() * h_xyz_b;
}

// Finds point indices within +- x, +- y, +- z
std::tuple<std::vector<int>, Eigen::Vector3d> AxisAlignedBoxFilter(
    const Eigen::Matrix3Xd& points, double x, double y, double z);

Eigen::Matrix3Xd SelectPoints(const Eigen::MatrixXd& points,
                              const std::vector<int>& indices);

Eigen::Map<const Eigen::MatrixXd> PointCloudGetData(
    const perception::PointCloud& cloud, const std::string& name);

// Save a 3xN point cloud as a ply file at the given path.
void SavePly(const std::string& ply_path, const Eigen::MatrixXd& points);
}  // namespace farm_ng::perception
#endif
