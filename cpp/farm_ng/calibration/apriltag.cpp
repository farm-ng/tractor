#include "farm_ng/calibration/apriltag.h"

#include "glog/logging.h"
namespace farm_ng {
std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection) {
  CHECK(detection.tag_size() > 0.0);
  double half_size = detection.tag_size() / 2.0;
  return std::array<Eigen::Vector3d, 4>(
      {Eigen::Vector3d(-half_size, -half_size, 0),
       Eigen::Vector3d(half_size, -half_size, 0.0),
       Eigen::Vector3d(half_size, half_size, 0.0),
       Eigen::Vector3d(-half_size, half_size, 0.0)});
}

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection) {
  return std::array<Eigen::Vector2d, 4>(
      {Eigen::Vector2d(detection.p(0).x(), detection.p(0).y()),
       Eigen::Vector2d(detection.p(1).x(), detection.p(1).y()),
       Eigen::Vector2d(detection.p(2).x(), detection.p(2).y()),
       Eigen::Vector2d(detection.p(3).x(), detection.p(3).y())});
}
}  // namespace farm_ng
