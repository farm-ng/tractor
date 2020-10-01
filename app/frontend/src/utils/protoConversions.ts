import {
  Vec3 as Vec3Proto,
  Quaternion as QuaternionProto,
  SE3Pose as SE3PoseProto
} from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import { Quaternion, Vector3, Matrix4 } from "three";

export function toVector3(v?: Vec3Proto): Vector3 {
  return v ? new Vector3(v.x, v.y, v.z) : new Vector3();
}

export function toQuaternion(q?: QuaternionProto): Quaternion {
  return q ? new Quaternion(q.x, q.y, q.z, q.w) : new Quaternion();
}

export function matrix4ToSE3Pose(m: Matrix4): SE3PoseProto {
  const t = new Vector3();
  const q = new Quaternion();
  const s = new Vector3();
  m.decompose(t, q, s);
  return SE3PoseProto.fromJSON({
    position: Vec3Proto.fromJSON({ x: t.x, y: t.y, z: t.z }),
    rotation: QuaternionProto.fromJSON({ x: q.x, y: q.y, z: q.z, w: q.w })
  });
}

export function se3PoseToMatrix4(pose: SE3PoseProto): Matrix4 {
  const m = new Matrix4();
  const { x: tx, y: ty, z: tz } = pose.position || { x: 0, y: 0, z: 0 };
  const { x: rx, y: ry, z: rz, w: rw } = pose.rotation || {
    x: 0,
    y: 0,
    z: 0,
    w: 1
  };
  m.compose(
    new Vector3(tx, ty, tz),
    new Quaternion(rx, ry, rz, rw),
    new Vector3(1, 1, 1)
  );
  return m;
}
