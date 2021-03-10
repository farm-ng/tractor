#ifndef FARM_NG_SOPHUS_PROTOBUF_H_
#define FARM_NG_SOPHUS_PROTOBUF_H_

#include <sophus/se3.hpp>

#include "farm_ng/perception/geometry.pb.h"

namespace farm_ng {
namespace perception {

inline void EigenToProto(const Sophus::SE3d::TranslationType& t, Vec3* pt) {
  pt->set_x(t.x());
  pt->set_y(t.y());
  pt->set_z(t.z());
}

inline void EigenToProto(const Sophus::SE3d::QuaternionType& q,
                         Quaternion* pq) {
  pq->set_x(q.x());
  pq->set_y(q.y());
  pq->set_z(q.z());
  pq->set_w(q.w());
}

inline void SophusToProto(const Sophus::SE3d& se3, SE3Pose* proto) {
  EigenToProto(se3.unit_quaternion(), proto->mutable_rotation());
  EigenToProto(se3.translation(), proto->mutable_position());
}

inline void SophusToProto(const Sophus::SE3d& se3, const std::string& frame_a,
                          const std::string& frame_b, NamedSE3Pose* proto) {
  SophusToProto(se3, proto->mutable_a_pose_b());
  proto->set_frame_a(frame_a);
  proto->set_frame_b(frame_b);
}

inline void SophusToProto(const Sophus::SE3d& se3,
                          const google::protobuf::Timestamp& stamp,
                          SE3Pose* proto) {
  EigenToProto(se3.unit_quaternion(), proto->mutable_rotation());
  EigenToProto(se3.translation(), proto->mutable_position());
  proto->mutable_stamp()->CopyFrom(stamp);
}

inline void SophusToProto(const Sophus::SE3d& se3,
                          const google::protobuf::Timestamp& stamp,
                          const std::string& frame_a,
                          const std::string& frame_b, NamedSE3Pose* proto) {
  SophusToProto(se3, stamp, proto->mutable_a_pose_b());
  proto->set_frame_a(frame_a);
  proto->set_frame_b(frame_b);
}

inline void ProtoToEigen(const Vec3& pt, Sophus::SE3d::TranslationType* t) {
  t->x() = pt.x();
  t->y() = pt.y();
  t->z() = pt.z();
}

inline Eigen::Vector3d ProtoToEigen(const Vec3& pt) {
  return Eigen::Vector3d(pt.x(), pt.y(), pt.z());
}

inline void ProtoToEigen(const Quaternion& pq,
                         Sophus::SE3d::QuaternionType* q) {
  q->x() = pq.x();
  q->y() = pq.y();
  q->z() = pq.z();
  q->w() = pq.w();
}

inline void ProtoToSophus(const SE3Pose& ppose, Sophus::SE3d* pose) {
  Sophus::SE3d::QuaternionType q;
  ProtoToEigen(ppose.rotation(), &q);
  pose->setQuaternion(q);
  ProtoToEigen(ppose.position(), &(pose->translation()));
}

inline Sophus::SE3d ProtoToSophus(const SE3Pose& ppose) {
  Sophus::SE3d pose;
  ProtoToSophus(ppose, &pose);
  return pose;
}

inline Sophus::SE3d ProtoToSophus(const NamedSE3Pose& ppose,
                                  const std::string& frame_a,
                                  const std::string& frame_b) {
  if (ppose.frame_a() == frame_a && ppose.frame_b() == frame_b) {
    return ProtoToSophus(ppose.a_pose_b());
  }
  if (ppose.frame_b() == frame_a && ppose.frame_b() == frame_a) {
    return ProtoToSophus(ppose.a_pose_b()).inverse();
  }
  LOG(FATAL) << "Pose doesn't contain: " << frame_a << " or " << frame_b
             << ppose.ShortDebugString();
  return Sophus::SE3d();
}

inline SE3Pose SophusToProto(const Sophus::SE3d& pose) {
  SE3Pose ppose;
  SophusToProto(pose, &ppose);
  return ppose;
}

inline NamedSE3Pose Inverse(NamedSE3Pose ppose) {
  SophusToProto(ProtoToSophus(ppose.a_pose_b()).inverse(),
                ppose.mutable_a_pose_b());
  std::string frame_a = ppose.frame_a();
  ppose.set_frame_a(ppose.frame_b());
  ppose.set_frame_b(frame_a);
  return ppose;
}

inline NamedSE3Pose Multiply(const NamedSE3Pose& pose0,
                             const NamedSE3Pose& pose1) {
  CHECK_EQ(pose0.frame_b(), pose1.frame_a());
  NamedSE3Pose out_pose;
  SophusToProto(
      ProtoToSophus(pose0.a_pose_b()) * ProtoToSophus(pose1.a_pose_b()),
      out_pose.mutable_a_pose_b());
  out_pose.set_frame_a(pose0.frame_a());
  out_pose.set_frame_b(pose1.frame_b());
  return out_pose;
}

}  // namespace perception
}  // namespace farm_ng
#endif  // FARM_NG_SOPHUS_PROTOBUF_H_
