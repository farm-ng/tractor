#ifndef FARM_NG_CALIBRATION_CAMERA_RIG_APRILTAG_RIG_ROBOT_EXTRINSICS_COST_FUNCTOR_H_
#define FARM_NG_CALIBRATION_CAMERA_RIG_APRILTAG_RIG_ROBOT_EXTRINSICS_COST_FUNCTOR_H_
#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/pose_graph.h"

namespace farm_ng {
namespace calibration {

using perception::CameraModel;
using perception::SE3Map;

struct CameraRigApriltagRigRobotExtrinsicsCostFunctor {
  CameraRigApriltagRigRobotExtrinsicsCostFunctor(
      const CameraModel& camera, const perception::ApriltagDetection& detection,
      Sophus::SE3d base_pose_link, SE3Map camera_pose_camera_rig,
      SE3Map tag_rig_pose_tag, SE3Map base_pose_camera_rig,
      SE3Map link_pose_tag_rig, double depth_scale = 100.0)
      : camera_(camera),
        points_tag_(PointsTag(detection)),
        points_image_(PointsImage(detection)),
        depths_camera_({0.0}),
        base_pose_link_(base_pose_link),
        camera_pose_camera_rig_(camera_pose_camera_rig),
        tag_rig_pose_tag_(tag_rig_pose_tag),
        base_pose_camera_rig_(base_pose_camera_rig),
        link_pose_tag_rig_(link_pose_tag_rig),
        depth_scale_(depth_scale) {
    CHECK(detection.depth_size() == 0 || detection.depth_size() == 4)
        << detection.ShortDebugString();
    if (detection.depth_size() == 4) {
      for (int i = 0; i < detection.depth_size(); ++i) {
        if (detection.depth(i).IsInitialized()) {
          depths_camera_[i] = detection.depth(i).value();
        }
      }
    }
  }

  template <class T>
  Eigen::Matrix<T, 4, 3> TransformToCamera(
      T const* const raw_camera_pose_camera_rig,
      T const* const raw_tag_rig_pose_tag,
      T const* const raw_base_pose_camera_rig,
      T const* const raw_link_pose_tag_rig) const {
    auto camera_pose_base =
        camera_pose_camera_rig_.Map(raw_camera_pose_camera_rig) *
        base_pose_camera_rig_.Map(raw_base_pose_camera_rig).inverse();

    auto link_pose_tag = link_pose_tag_rig_.Map(raw_link_pose_tag_rig) *
                         tag_rig_pose_tag_.Map(raw_tag_rig_pose_tag);

    auto camera_pose_tag =
        camera_pose_base * base_pose_link_.cast<T>() * link_pose_tag;
    Eigen::Matrix<T, 4, 3> points_camera;
    for (int i = 0; i < 4; ++i) {
      points_camera.row(i) = camera_pose_tag * points_tag_[i].cast<T>();
    }
    return points_camera;
  }

  template <class T>
  bool operator()(T const* const raw_camera_pose_camera_rig,
                  T const* const raw_tag_rig_pose_tag,
                  T const* const raw_base_pose_camera_rig,
                  T const* const raw_link_pose_tag_rig,
                  T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 4, 3>> residuals(raw_residuals);
    Eigen::Matrix<T, 4, 3> points_camera =
        TransformToCamera(raw_camera_pose_camera_rig, raw_tag_rig_pose_tag,
                          raw_base_pose_camera_rig, raw_link_pose_tag_rig);

    for (int i = 0; i < 4; ++i) {
      residuals.row(i).template head<2>() =
          points_image_[i].cast<T>() -
          perception::ProjectPointToPixel<T>(camera_, points_camera.row(i));
      if (depths_camera_[i] > 0.0 && points_camera(i, 2) > T(0.0)) {
        residuals(i, 2) =
            T(depth_scale_) * (points_camera(i, 2) - T(depths_camera_[i]));
      } else {
        residuals(i, 2) = T(0.0);
      }
    }

    return true;
  }
  CameraModel camera_;
  std::array<Eigen::Vector3d, 4> points_tag_;
  std::array<Eigen::Vector2d, 4> points_image_;
  std::array<double, 4> depths_camera_;
  Sophus::SE3d base_pose_link_;

  SE3Map camera_pose_camera_rig_;
  SE3Map tag_rig_pose_tag_;
  SE3Map base_pose_camera_rig_;
  SE3Map link_pose_tag_rig_;

  double depth_scale_;
};

}  // namespace calibration
}  // namespace farm_ng
#endif
