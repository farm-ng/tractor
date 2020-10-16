import {
  ApriltagConfig,
  ApriltagDetections,
  ApriltagRig,
  TagConfig
} from "../../genproto/farm_ng_proto/tractor/v1/apriltag";
import {
  NamedSE3Pose,
  SE3Pose,
  Vec2
} from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import {
  BaseToCameraInitialization,
  BaseToCameraModel,
  CalibrationParameter,
  CalibratorCommand,
  CalibratorStatus,
  MonocularApriltagRigModel,
  ViewInitialization
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
import {
  TractorConfig,
  TractorState
} from "../../genproto/farm_ng_proto/tractor/v1/tractor";
import { Message } from "../types/common";
import {
  ProgramSupervisorStatus,
  StartProgramRequest,
  StopProgramRequest
} from "../../genproto/farm_ng_proto/tractor/v1/program_supervisor";
import {
  CaptureCalibrationDatasetConfiguration,
  CaptureCalibrationDatasetResult,
  CaptureCalibrationDatasetStatus
} from "../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import {
  CaptureVideoDatasetConfiguration,
  CaptureVideoDatasetResult,
  CaptureVideoDatasetStatus
} from "../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";
import {
  CalibrateApriltagRigConfiguration,
  CalibrateApriltagRigResult,
  CalibrateApriltagRigStatus
} from "../../genproto/farm_ng_proto/tractor/v1/calibrate_apriltag_rig";
import {
  CalibrateBaseToCameraConfiguration,
  CalibrateBaseToCameraResult,
  CalibrateBaseToCameraStatus
} from "../../genproto/farm_ng_proto/tractor/v1/calibrate_base_to_camera";
import { Event as BusEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";

export type EventType =
  | BusEvent
  | SteeringCommand
  | TrackingCameraPoseFrame
  | TrackingCameraMotionFrame
  | SE3Pose
  | NamedSE3Pose
  | MotorControllerState
  | ApriltagDetections
  | TractorState
  | Announce
  | Vec2
  | Image
  | TractorConfig
  | ApriltagConfig
  | TagConfig
  | CalibrationParameter
  | ViewInitialization
  | LoggingCommand
  | TrackingCameraCommand
  | CalibratorCommand
  | CalibratorStatus
  | LoggingStatus
  | ProgramSupervisorStatus
  | StartProgramRequest
  | ApriltagRig
  | StopProgramRequest
  | MonocularApriltagRigModel
  | BaseToCameraModel
  | BaseToCameraInitialization
  | CaptureCalibrationDatasetConfiguration
  | CaptureCalibrationDatasetStatus
  | CaptureCalibrationDatasetResult
  | CaptureVideoDatasetConfiguration
  | CaptureVideoDatasetStatus
  | CaptureVideoDatasetResult
  | CalibrateApriltagRigConfiguration
  | CalibrateApriltagRigStatus
  | CalibrateApriltagRigResult
  | CalibrateBaseToCameraConfiguration
  | CalibrateBaseToCameraStatus
  | CalibrateBaseToCameraResult;

// Infer the keys, but restrict values to Message<EventType>
// https://stackoverflow.com/a/54598743
const inferKeys = <T>(
  o: { [K in keyof T]: Message<EventType> }
): { [K in keyof T]: Message<EventType> } => o;

export const eventRegistry = inferKeys({
  "type.googleapis.com/farm_ng_proto.tractor.v1.Event": BusEvent,
  "type.googleapis.com/farm_ng_proto.tractor.v1.SteeringCommand": SteeringCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraPoseFrame": TrackingCameraPoseFrame,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraMotionFrame": TrackingCameraMotionFrame,
  "type.googleapis.com/farm_ng_proto.tractor.v1.SE3Pose": SE3Pose,
  "type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose": NamedSE3Pose,
  "type.googleapis.com/farm_ng_proto.tractor.v1.MotorControllerState": MotorControllerState,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagDetections": ApriltagDetections,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagRig": ApriltagRig,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TractorState": TractorState,
  "type.googleapis.com/farm_ng_proto.tractor.v1.Announce": Announce,
  "type.googleapis.com/farm_ng_proto.tractor.v1.Vec2": Vec2,
  "type.googleapis.com/farm_ng_proto.tractor.v1.Image": Image,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TractorConfig": TractorConfig,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagConfig": ApriltagConfig,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TagConfig": TagConfig,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ViewInitialization": ViewInitialization,
  "type.googleapis.com/farm_ng_proto.tractor.v1.BaseToCameraInitialization": BaseToCameraInitialization,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrationParameter": CalibrationParameter,
  "type.googleapis.com/farm_ng_proto.tractor.v1.LoggingCommand": LoggingCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraCommand": TrackingCameraCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibratorCommand": CalibratorCommand,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibratorStatus": CalibratorStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.LoggingStatus": LoggingStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.MonocularApriltagRigModel": MonocularApriltagRigModel,
  "type.googleapis.com/farm_ng_proto.tractor.v1.BaseToCameraModel": BaseToCameraModel,
  "type.googleapis.com/farm_ng_proto.tractor.v1.ProgramSupervisorStatus": ProgramSupervisorStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.StartProgramRequest": StartProgramRequest,
  "type.googleapis.com/farm_ng_proto.tractor.v1.StopProgramRequest": StopProgramRequest,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetConfiguration": CaptureCalibrationDatasetConfiguration,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetStatus": CaptureCalibrationDatasetStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetResult": CaptureCalibrationDatasetResult,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetConfiguration": CaptureVideoDatasetConfiguration,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetStatus": CaptureVideoDatasetStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetResult": CaptureVideoDatasetResult,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigConfiguration": CalibrateApriltagRigConfiguration,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigStatus": CalibrateApriltagRigStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigResult": CalibrateApriltagRigResult,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraConfiguration": CalibrateBaseToCameraConfiguration,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraStatus": CalibrateBaseToCameraStatus,
  "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraResult": CalibrateBaseToCameraResult
});

export const eventTypeIds = Object.keys(
  eventRegistry
) as (keyof typeof eventRegistry)[];
export type EventTypeId = typeof eventTypeIds[number];
