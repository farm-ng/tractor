#ifndef FARM_NG_SOPHUS_PROTOBUF_H_
#define FARM_NG_SOPHUS_PROTOBUF_H_

#include <sophus/se3.hpp>
#include "farm_ng_proto/tractor/v1/geometry.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::Quaternion;
using farm_ng_proto::tractor::v1::SE3Pose;
using farm_ng_proto::tractor::v1::Vec3;

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

inline void ProtoToEigen(const Vec3& pt, Sophus::SE3d::TranslationType* t) {
  t->x() = pt.x();
  t->y() = pt.y();
  t->z() = pt.z();
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
}  // namespace farm_ng
#endif  // FARM_NG_SOPHUS_PROTOBUF_H_
