#ifndef FARM_NG_PERCEPTION_ROBOT_ARM_FK_H_
#define FARM_NG_PERCEPTION_ROBOT_ARM_FK_H_
#include "farm_ng/perception/kinematics.pb.h"
#include "farm_ng/perception/sophus_protobuf.h"

namespace farm_ng {
namespace perception {
using Sophus::SE3d;

class RobotLinkFK {
 public:
  explicit RobotLinkFK(RobotLink link)
      : parent(link.parent_pose_self().frame_a()),
        self(link.parent_pose_self().frame_b()),
        joint_name(link.joint_name()),
        joint_axis(link.joint_axis()) {
    ProtoToSophus(link.parent_pose_self().a_pose_b(), &parent_pose_self);
  }

  RobotLinkFK(std::string parent, std::string self, SE3d parent_pose_self,
              std::string joint_name, RobotLink::JointAxis joint_axis)
      : parent(parent),
        self(self),
        parent_pose_self(parent_pose_self),
        joint_name(joint_name),
        joint_axis(joint_axis) {}

  template <typename T>
  Sophus::SE3<T> JointFK(const T& value) const {
    switch (joint_axis) {
      case RobotLink::JOINT_AXIS_RX:
        return Sophus::SE3<T>::rotX(value);
      case RobotLink::JOINT_AXIS_RY:
        return Sophus::SE3<T>::rotY(value);
      case RobotLink::JOINT_AXIS_RZ:
      default:
        return Sophus::SE3<T>::rotZ(value);
    }
  }

  template <typename T>
  Sophus::SE3<T> FK(const T& value) const {
    return parent_pose_self.cast<T>() * JointFK<T>(value);
  }

  RobotLink ToProto() const {
    RobotLink link;
    SophusToProto(parent_pose_self, parent, self,
                  link.mutable_parent_pose_self());
    link.set_joint_name(joint_name);
    link.set_joint_axis(joint_axis);
    return link;
  }
  std::string parent;
  std::string self;
  SE3d parent_pose_self;
  std::string joint_name;
  RobotLink::JointAxis joint_axis;
};

class RobotArmFK6dof {
 public:
  RobotArmFK6dof(RobotArm robot_arm) {
    CHECK_EQ(robot_arm.links_size(), 6);
    for (int i = 0; i < robot_arm.links_size() - 1; ++i) {
      CHECK_EQ(robot_arm.links(i).parent_pose_self().frame_b(),
               robot_arm.links(i + 1).parent_pose_self().frame_a());
    }
    for (auto link : robot_arm.links()) {
      links_.push_back(RobotLinkFK(link));
    }
  }

  RobotArmFK6dof(std::vector<RobotLinkFK> links) : links_(links) {
    CHECK_EQ(links.size(), 6);
    for (size_t i = 0; i < links.size() - 1; ++i) {
      CHECK_EQ(links[i].self, links[i + 1].parent);
    }
  }
  template <typename T>
  Sophus::SE3<T> FK(const Eigen::Matrix<T, 6, 1>& joint_values,
                    const Eigen::Matrix<T, 6, 1>& joint_offsets) const {
    Sophus::SE3<T> base_pose_ee = Sophus::SE3<T>::rotX(T(0));  // identity
    for (size_t i = 0; i < links_.size(); ++i) {
      base_pose_ee =
          base_pose_ee * links_[i].FK(joint_values(i) + joint_offsets(i));
    }
    return base_pose_ee;
  }

  RobotArm ToProto() const {
    RobotArm robot_arm;
    for (auto link : links_) {
      robot_arm.add_links()->CopyFrom(link.ToProto());
    }
    return robot_arm;
  }

 private:
  std::vector<RobotLinkFK> links_;
};
}  // namespace perception
}  // namespace farm_ng
#endif
