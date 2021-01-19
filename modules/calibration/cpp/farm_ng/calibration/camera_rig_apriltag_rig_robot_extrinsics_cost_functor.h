#ifndef FARM_NG_CALIBRATION_CAMERA_RIG_APRILTAG_RIG_ROBOT_EXTRINSICS_COST_FUNCTOR_H_
#define FARM_NG_CALIBRATION_CAMERA_RIG_APRILTAG_RIG_ROBOT_EXTRINSICS_COST_FUNCTOR_H_
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/pose_graph.h"

namespace farm_ng {
namespace calibration {

using perception::CameraModel;
using perception::SE3Map;

struct CameraRigApriltagRigRobotExtrinsicsCostFunctor {
  CameraRigApriltagRigRobotExtrinsicsCostFunctor(
      const CameraModel& camera, std::array<Eigen::Vector3d, 4> points_tag,
      std::array<Eigen::Vector2d, 4> points_image, Sophus::SE3d base_pose_link,
      SE3Map camera_pose_camera_rig, SE3Map tag_rig_pose_tag,
      SE3Map camera_rig_pose_tag_rig, SE3Map base_pose_camera_rig,
      SE3Map link_pose_tag_rig)
      : camera_(camera),
        points_tag_(points_tag),
        points_image_(points_image),
        base_pose_link_(base_pose_link),
        camera_pose_camera_rig_(camera_pose_camera_rig),
        tag_rig_pose_tag_(tag_rig_pose_tag),
        camera_rig_pose_tag_rig_(camera_rig_pose_tag_rig),
        base_pose_camera_rig_(base_pose_camera_rig),
        link_pose_tag_rig_(link_pose_tag_rig) {}

  template <class T>
  Eigen::Matrix<T, 4, 2> ProjectThroughBasePoseLink(
      T const* const raw_camera_pose_camera_rig,
      T const* const raw_tag_rig_pose_tag,
      T const* const raw_camera_rig_pose_tag_rig,
      T const* const raw_base_pose_camera_rig,
      T const* const raw_link_pose_tag_rig) const {}

  template <class T>
  bool operator()(T const* const raw_camera_pose_camera_rig,
                  T const* const raw_tag_rig_pose_tag,
                  T const* const raw_camera_rig_pose_tag_rig,

                  T const* const raw_base_pose_camera_rig,
                  T const* const raw_link_pose_tag_rig,
                  T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);

    auto camera_pose_camera_rig =
        camera_pose_camera_rig_.Map(raw_camera_pose_camera_rig);
    auto tag_rig_pose_tag = tag_rig_pose_tag_.Map(raw_tag_rig_pose_tag);
    auto camera_rig_pose_tag_rig =
        tag_rig_pose_tag_.Map(raw_camera_rig_pose_tag_rig);
    auto base_pose_camera_rig =
        base_pose_camera_rig_.Map(raw_base_pose_camera_rig);
    auto link_pose_tag_rig = link_pose_tag_rig_.Map(raw_link_pose_tag_rig);
    Sophus::SE3<T> tag_pose_tag =
        tag_rig_pose_tag.inverse() * camera_rig_pose_tag_rig.inverse() *
        camera_pose_camera_rig * base_pose_camera_rig.inverse() *
        base_pose_link_.cast<T>() * link_pose_tag_rig * tag_rig_pose_tag;
    residuals = tag_pose_tag.log();

    return true;
  }
  CameraModel camera_;
  std::array<Eigen::Vector3d, 4> points_tag_;
  std::array<Eigen::Vector2d, 4> points_image_;
  Sophus::SE3d base_pose_link_;

  SE3Map camera_pose_camera_rig_;
  SE3Map tag_rig_pose_tag_;
  SE3Map camera_rig_pose_tag_rig_;
  SE3Map base_pose_camera_rig_;
  SE3Map link_pose_tag_rig_;
};

}  // namespace calibration
}  // namespace farm_ng
#endif
