import {
  ApriltagConfig,
  ApriltagDetections,
  ApriltagRig,
  TagConfig,
} from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import {
  NamedSE3Pose,
  SE3Pose,
  Vec2,
} from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import {
  BaseToCameraInitialization,
  BaseToCameraModel,
  CalibrationParameter,
  CalibratorCommand,
  CalibratorStatus,
  MonocularApriltagRigModel,
  MultiViewApriltagRigModel,
  ViewInitialization,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import {
  Announce,
  LoggingCommand,
  LoggingStatus,
} from "@farm-ng/genproto-core/farm_ng/core/io";
import { Image } from "@farm-ng/genproto-perception/farm_ng/perception/image";
import { IntrinsicModel } from "@farm-ng/genproto-calibration/farm_ng/calibration/intrinsic_model";
import { MotorControllerState } from "@farm-ng/genproto-tractor/farm_ng/tractor/motor";
import { SteeringCommand } from "@farm-ng/genproto-tractor/farm_ng/tractor/steering";
import {
  CameraPipelineCommand,
  CameraPipelineConfig,
  CameraConfig,
} from "@farm-ng/genproto-perception/farm_ng/perception/camera_pipeline";
import {
  TractorConfig,
  TractorState,
} from "@farm-ng/genproto-tractor/farm_ng/tractor/tractor";
import { Message } from "../types/common";
import {
  ProgramSupervisorStatus,
  StartProgramRequest,
  StopProgramRequest,
} from "@farm-ng/genproto-core/farm_ng/core/programd";
import {
  CalibrateIntrinsicsConfiguration,
  CalibrateIntrinsicsResult,
  CalibrateIntrinsicsStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_intrinsics";
import {
  CaptureVideoDatasetConfiguration,
  CaptureVideoDatasetResult,
  CaptureVideoDatasetStatus,
} from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import {
  CreateVideoDatasetConfiguration,
  CreateVideoDatasetResult,
  CreateVideoDatasetStatus,
  VideoFileCamera,
} from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import {
  DetectApriltagsConfiguration,
  DetectApriltagsStatus,
} from "@farm-ng/genproto-perception/farm_ng/perception/detect_apriltags";
import {
  CalibrateApriltagRigConfiguration,
  CalibrateApriltagRigResult,
  CalibrateApriltagRigStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import {
  CalibrateBaseToCameraConfiguration,
  CalibrateBaseToCameraResult,
  CalibrateBaseToCameraStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_base_to_camera";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import {
  CalibrateMultiViewApriltagRigConfiguration,
  CalibrateMultiViewApriltagRigResult,
  CalibrateMultiViewApriltagRigStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_multi_view_apriltag_rig";
import { CameraModel } from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";

export type EventType =
  | Announce
  | ApriltagConfig
  | ApriltagDetections
  | ApriltagRig
  | BaseToCameraInitialization
  | BaseToCameraModel
  | BusEvent
  | CalibrateApriltagRigConfiguration
  | CalibrateApriltagRigResult
  | CalibrateApriltagRigStatus
  | CalibrateBaseToCameraConfiguration
  | CalibrateBaseToCameraResult
  | CalibrateBaseToCameraStatus
  | CalibrateMultiViewApriltagRigConfiguration
  | CalibrateMultiViewApriltagRigResult
  | CalibrateMultiViewApriltagRigStatus
  | CalibrateIntrinsicsConfiguration
  | CalibrateIntrinsicsResult
  | CalibrateIntrinsicsStatus
  | CalibrationParameter
  | CalibratorCommand
  | CalibratorStatus
  | CameraConfig
  | CaptureVideoDatasetConfiguration
  | CaptureVideoDatasetResult
  | CaptureVideoDatasetStatus
  | CreateVideoDatasetConfiguration
  | CreateVideoDatasetResult
  | CreateVideoDatasetStatus
  | DetectApriltagsConfiguration
  | DetectApriltagsStatus
  | Image
  | IntrinsicModel
  | LoggingCommand
  | LoggingStatus
  | MonocularApriltagRigModel
  | MotorControllerState
  | MultiViewApriltagRigModel
  | NamedSE3Pose
  | ProgramSupervisorStatus
  | SE3Pose
  | StartProgramRequest
  | SteeringCommand
  | StopProgramRequest
  | TagConfig
  | CameraModel
  | CameraPipelineCommand
  | CameraPipelineConfig
  | TractorConfig
  | TractorState
  | Vec2
  | VideoFileCamera
  | ViewInitialization;

// Infer the keys, but restrict values to Message<EventType>
// https://stackoverflow.com/a/54598743
const inferKeys = <T>(
  o: { [K in keyof T]: Message<EventType> }
): { [K in keyof T]: Message<EventType> } => o;

export const eventRegistry = inferKeys({
  "type.googleapis.com/farm_ng.calibration.BaseToCameraInitialization": BaseToCameraInitialization,
  "type.googleapis.com/farm_ng.calibration.BaseToCameraModel": BaseToCameraModel,
  "type.googleapis.com/farm_ng.perception.CameraModel": CameraModel,
  "type.googleapis.com/farm_ng.calibration.IntrinsicModel": IntrinsicModel,
  "type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigConfiguration": CalibrateApriltagRigConfiguration,
  "type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigResult": CalibrateApriltagRigResult,
  "type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigStatus": CalibrateApriltagRigStatus,
  "type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraConfiguration": CalibrateBaseToCameraConfiguration,
  "type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraResult": CalibrateBaseToCameraResult,
  "type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraStatus": CalibrateBaseToCameraStatus,
  "type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsConfiguration": CalibrateIntrinsicsConfiguration,
  "type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsResult": CalibrateIntrinsicsResult,
  "type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsStatus": CalibrateIntrinsicsStatus,
  "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigConfiguration": CalibrateMultiViewApriltagRigConfiguration,
  "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigResult": CalibrateMultiViewApriltagRigResult,
  "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigStatus": CalibrateMultiViewApriltagRigStatus,
  "type.googleapis.com/farm_ng.calibration.CalibrationParameter": CalibrationParameter,
  "type.googleapis.com/farm_ng.calibration.CalibratorCommand": CalibratorCommand,
  "type.googleapis.com/farm_ng.calibration.CalibratorStatus": CalibratorStatus,
  "type.googleapis.com/farm_ng.calibration.MonocularApriltagRigModel": MonocularApriltagRigModel,
  "type.googleapis.com/farm_ng.calibration.MultiViewApriltagRigModel": MultiViewApriltagRigModel,
  "type.googleapis.com/farm_ng.calibration.ViewInitialization": ViewInitialization,
  "type.googleapis.com/farm_ng.core.Announce": Announce,
  "type.googleapis.com/farm_ng.core.Event": BusEvent,
  "type.googleapis.com/farm_ng.core.LoggingCommand": LoggingCommand,
  "type.googleapis.com/farm_ng.core.LoggingStatus": LoggingStatus,
  "type.googleapis.com/farm_ng.core.ProgramSupervisorStatus": ProgramSupervisorStatus,
  "type.googleapis.com/farm_ng.core.StartProgramRequest": StartProgramRequest,
  "type.googleapis.com/farm_ng.core.StopProgramRequest": StopProgramRequest,
  "type.googleapis.com/farm_ng.perception.ApriltagConfig": ApriltagConfig,
  "type.googleapis.com/farm_ng.perception.ApriltagDetections": ApriltagDetections,
  "type.googleapis.com/farm_ng.perception.ApriltagRig": ApriltagRig,
  "type.googleapis.com/farm_ng.perception.CameraConfig": CameraConfig,
  "type.googleapis.com/farm_ng.perception.CameraPipelineCommand": CameraPipelineCommand,
  "type.googleapis.com/farm_ng.perception.CameraPipelineConfig": CameraPipelineConfig,
  "type.googleapis.com/farm_ng.perception.CaptureVideoDatasetConfiguration": CaptureVideoDatasetConfiguration,
  "type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult": CaptureVideoDatasetResult,
  "type.googleapis.com/farm_ng.perception.CaptureVideoDatasetStatus": CaptureVideoDatasetStatus,
  "type.googleapis.com/farm_ng.perception.CreateVideoDatasetConfiguration": CreateVideoDatasetConfiguration,
  "type.googleapis.com/farm_ng.perception.CreateVideoDatasetResult": CreateVideoDatasetResult,
  "type.googleapis.com/farm_ng.perception.CreateVideoDatasetStatus": CreateVideoDatasetStatus,
  "type.googleapis.com/farm_ng.perception.DetectApriltagsConfiguration": DetectApriltagsConfiguration,
  "type.googleapis.com/farm_ng.perception.DetectApriltagsStatus": DetectApriltagsStatus,
  "type.googleapis.com/farm_ng.perception.Image": Image,
  "type.googleapis.com/farm_ng.perception.NamedSE3Pose": NamedSE3Pose,
  "type.googleapis.com/farm_ng.perception.SE3Pose": SE3Pose,
  "type.googleapis.com/farm_ng.perception.TagConfig": TagConfig,
  "type.googleapis.com/farm_ng.perception.Vec2": Vec2,
  "type.googleapis.com/farm_ng.perception.VideoFileCamera": VideoFileCamera,
  "type.googleapis.com/farm_ng.tractor.MotorControllerState": MotorControllerState,
  "type.googleapis.com/farm_ng.tractor.SteeringCommand": SteeringCommand,
  "type.googleapis.com/farm_ng.tractor.TractorConfig": TractorConfig,
  "type.googleapis.com/farm_ng.tractor.TractorState": TractorState,
});

export const eventTypeIds = Object.keys(
  eventRegistry
) as (keyof typeof eventRegistry)[];
export type EventTypeId = typeof eventTypeIds[number];
