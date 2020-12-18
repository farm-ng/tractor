import {
  Vec3 as Vec3Proto,
  Quaternion as QuaternionProto,
  SE3Pose as SE3PoseProto,
  NamedSE3Pose,
} from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import { Quaternion, Vector3, Matrix4, Euler } from "three";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { EventTypeId, eventTypeIds } from "../registry/events";
import { CameraModel } from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";
import { rad2deg } from "./geometry";

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
    rotation: QuaternionProto.fromJSON({ x: q.x, y: q.y, z: q.z, w: q.w }),
  });
}

export function se3PoseToMatrix4(pose: SE3PoseProto): Matrix4 {
  const m = new Matrix4();
  const { x: tx, y: ty, z: tz } = pose.position || { x: 0, y: 0, z: 0 };
  const { x: rx, y: ry, z: rz, w: rw } = pose.rotation || {
    x: 0,
    y: 0,
    z: 0,
    w: 1,
  };
  m.compose(
    new Vector3(tx, ty, tz),
    new Quaternion(rx, ry, rz, rw),
    new Vector3(1, 1, 1)
  );
  return m;
}

export function openCVPoseToThreeJSPose(pose: NamedSE3Pose): NamedSE3Pose {
  if (!pose.aPoseB) {
    console.warn("Could not convert openCV pose to threeJS pose: ", pose);
    return pose;
  }
  const opencvTthreejs = new Matrix4().makeRotationFromEuler(
    new Euler(Math.PI, 0, 0)
  );
  const cameraTransform = se3PoseToMatrix4(pose.aPoseB);
  return {
    ...pose,
    aPoseB: matrix4ToSE3Pose(cameraTransform.multiply(opencvTthreejs)),
  };
}

export function cameraModelToThreeJSFOV(cameraModel: CameraModel): number {
  return rad2deg(2 * Math.atan(cameraModel.imageHeight / (2 * cameraModel.fy)));
}

const resourceFormats = ["application/json", "application/protobuf"] as const;
type ResourceFormat = typeof resourceFormats[number];
export function parseResourceContentType(
  resource: Resource
): [ResourceFormat | null, EventTypeId | null] {
  const [format, typeUrl] = resource.contentType.split("; type=");
  if (!eventTypeIds.includes(typeUrl as EventTypeId)) {
    return [null, null];
  }
  if (!resourceFormats.includes(format as ResourceFormat)) {
    return [null, null];
  }
  return [format as ResourceFormat, typeUrl as EventTypeId];
}
