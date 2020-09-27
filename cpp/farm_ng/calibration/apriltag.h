#ifndef FARM_NG_CALIBRATION_APRILTAG_H_
#define FARM_NG_CALIBRATION_APRILTAG_H_
#include <array>

#include <Eigen/Dense>

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::ApriltagDetection;

std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection);

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection);
}  // namespace farm_ng
#endif
