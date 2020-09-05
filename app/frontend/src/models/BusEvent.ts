import { SteeringCommand } from "../../genproto/farm_ng_proto/tractor/v1/steering";
import {
  TrackingCameraPoseFrame,
  TrackingCameraMotionFrame
} from "../../genproto/farm_ng_proto/tractor/v1/tracking_camera";
import { NamedSE3Pose } from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import { MotorControllerState } from "../../genproto/farm_ng_proto/tractor/v1/motor";
import { ApriltagDetections } from "../../genproto/farm_ng_proto/tractor/v1/apriltag";

export type BusEvent =
  | SteeringCommand
  | TrackingCameraPoseFrame
  | TrackingCameraMotionFrame
  | NamedSE3Pose
  | MotorControllerState
  | ApriltagDetections;
