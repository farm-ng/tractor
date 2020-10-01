import {
  ApriltagDetections,
  ApriltagRig
} from "../../genproto/farm_ng_proto/tractor/v1/apriltag";
import {
  NamedSE3Pose,
  Vec2
} from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import {
  CalibratorCommand,
  CalibratorStatus
} from "../../genproto/farm_ng_proto/tractor/v1/calibrator";
import {
  Announce,
  LoggingCommand,
  LoggingStatus
} from "../../genproto/farm_ng_proto/tractor/v1/io";
import { Image } from "../../genproto/farm_ng_proto/tractor/v1/image";
import { MotorControllerState } from "../../genproto/farm_ng_proto/tractor/v1/motor";
import { SteeringCommand } from "../../genproto/farm_ng_proto/tractor/v1/steering";
import {
  TrackingCameraPoseFrame,
  TrackingCameraMotionFrame,
  TrackingCameraCommand
} from "../../genproto/farm_ng_proto/tractor/v1/tracking_camera";
import { TractorState } from "../../genproto/farm_ng_proto/tractor/v1/tractor";
import { Message } from "../types/common";
import {
  ProgramSupervisorStatus,
  StartProgramRequest,
  StopProgramRequest
} from "../../genproto/farm_ng_proto/tractor/v1/program_supervisor";

export type EventType =
  | SteeringCommand
  | TrackingCameraPoseFrame
  | TrackingCameraMotionFrame
  | NamedSE3Pose
  | MotorControllerState
  | ApriltagDetections
  | TractorState
  | Announce
  | Vec2
  | Image
  | LoggingCommand
  | TrackingCameraCommand
  | CalibratorCommand
  | CalibratorStatus
  | LoggingStatus
  | ProgramSupervisorStatus
  | StartProgramRequest
  | ApriltagRig
  | StopProgramRequest;

// Infer the keys, but restrict values to Message<EventType>
// https://stackoverflow.com/a/54598743
const inferKeys = <T>(
  o: { [K in keyof T]: Message<EventType> }
): { [K in keyof T]: Message<EventType> } => o;

export const eventRegistry = inferKeys({
  "type.googleapis.com/farm_ng_proto.tractor.v1.SteeringCommand": SteeringCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraPoseFrame": TrackingCameraPoseFrame,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraMotionFrame": TrackingCameraMotionFrame,
  "type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose": NamedSE3Pose,
  "type.googleapis.com/farm_ng_proto.tractor.v1.MotorControllerState": MotorControllerState,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagDetections": ApriltagDetections,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagRig": ApriltagRig,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TractorState": TractorState,
  "type.googleapis.com/farm_ng_proto.tractor.v1.Announce": Announce,
  "type.googleapis.com/farm_ng_proto.tractor.v1.Vec2": Vec2,
  "type.googleapis.com/farm_ng_proto.tractor.v1.Image": Image,
  "type.googleapis.com/farm_ng_proto.tractor.v1.LoggingCommand": LoggingCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraCommand": TrackingCameraCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibratorCommand": CalibratorCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibratorStatus": CalibratorStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.LoggingStatus": LoggingStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ProgramSupervisorStatus": ProgramSupervisorStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.StartProgramRequest": StartProgramRequest,
  "type.googleapis.com/farm_ng_proto.tractor.v1.StopProgramRequest": StopProgramRequest
});

export const eventTypeIds = Object.keys(
  eventRegistry
) as (keyof typeof eventRegistry)[];
export type EventTypeId = typeof eventTypeIds[number];
