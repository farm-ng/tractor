import { CalibrateApriltagRigConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import { CalibrateBaseToCameraConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_base_to_camera";
import { CalibrateIntrinsicsConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_intrinsics";
import { CalibrateMultiViewApriltagRigConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_multi_view_apriltag_rig";
import { CalibrateRobotExtrinsicsConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_robot_extrinsics";
import { ValidateRobotExtrinsicsConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/validate_robot_extrinsics";
import { LogPlaybackConfiguration } from "@farm-ng/genproto-core/farm_ng/core/log_playback";
import { Resource } from "@farm-ng/genproto-perception/farm_ng/core/resource";
import { CaptureVideoDatasetConfiguration } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { CreateVideoDatasetConfiguration } from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import { DetectApriltagsConfiguration } from "@farm-ng/genproto-perception/farm_ng/perception/detect_apriltags";
import { CalibrateApriltagRigConfigurationVisualizer } from "../scope/visualizers/CalibrateApriltagRigConfiguration";
import { CalibrateBaseToCameraConfigurationVisualizer } from "../scope/visualizers/CalibrateBaseToCameraConfiguration";
import { CalibrateIntrinsicsConfigurationVisualizer } from "../scope/visualizers/CalibrateIntrinsicsConfiguration";
import { CalibrateMultiViewApriltagRigConfigurationVisualizer } from "../scope/visualizers/CalibrateMultiViewApriltagRigConfiguration";
import { CalibrateRobotExtrinsicsConfigurationVisualizer } from "../scope/visualizers/CalibrateRobotExtrinsicsConfiguration";
import { CaptureVideoDatasetConfigurationVisualizer } from "../scope/visualizers/CaptureVideoDatasetConfiguration";
import { CreateVideoDatasetConfigurationVisualizer } from "../scope/visualizers/CreateVideoDatasetConfiguration";
import { DetectApriltagsConfigurationVisualizer } from "../scope/visualizers/DetectApriltagsConfiguration";
import { LogPlaybackConfigurationVisualizer } from "../scope/visualizers/LogPlaybackConfiguration";
import { ResourceVisualizer } from "../scope/visualizers/Resource";
import { ValidateRobotExtrinsicsConfigurationVisualizer } from "../scope/visualizers/ValidateRobotExtrinsicsConfiguration";
import { makeStandardProgram } from "./makeStandardProgram";

export const CalibrateApriltagRigProgram = makeStandardProgram(
  "calibrate_apriltag_rig",
  CalibrateApriltagRigConfiguration,
  CalibrateApriltagRigConfigurationVisualizer.Form
);

export const CalibrateBaseToCameraProgram = makeStandardProgram(
  "calibrate_base_to_camera",
  CalibrateBaseToCameraConfiguration,
  CalibrateBaseToCameraConfigurationVisualizer.Form
);

export const CalibrateIntrinsicsProgram = makeStandardProgram(
  "calibrate_intrinsics",
  CalibrateIntrinsicsConfiguration,
  CalibrateIntrinsicsConfigurationVisualizer.Form
);

export const CalibrateMultiViewApriltagRigProgram = makeStandardProgram(
  "calibrate_multi_view_apriltag_rig",
  CalibrateMultiViewApriltagRigConfiguration,
  CalibrateMultiViewApriltagRigConfigurationVisualizer.Form
);

export const CalibrateRobotExtrinsicsProgram = makeStandardProgram(
  "calibrate_robot_extrinsics",
  CalibrateRobotExtrinsicsConfiguration,
  CalibrateRobotExtrinsicsConfigurationVisualizer.Form
);

export const CaptureRobotExtrinsicsDatasetProgram = makeStandardProgram(
  "capture_robot_extrinsics_dataset",
  Resource,
  ResourceVisualizer.Form
);

export const CaptureVideoDatasetProgram = makeStandardProgram(
  "capture_video_dataset",
  CaptureVideoDatasetConfiguration,
  CaptureVideoDatasetConfigurationVisualizer.Form
);

export const CreateVideoDatasetProgram = makeStandardProgram(
  "create_video_dataset",
  CreateVideoDatasetConfiguration,
  CreateVideoDatasetConfigurationVisualizer.Form
);

export const DetectApriltagsProgram = makeStandardProgram(
  "detect_apriltags",
  DetectApriltagsConfiguration,
  DetectApriltagsConfigurationVisualizer.Form
);

export const LogPlaybackProgram = makeStandardProgram(
  "log_playback",
  LogPlaybackConfiguration,
  LogPlaybackConfigurationVisualizer.Form
);

export const ValidateRobotExtrinsicsProgram = makeStandardProgram(
  "validate_robot_extrinsics",
  ValidateRobotExtrinsicsConfiguration,
  ValidateRobotExtrinsicsConfigurationVisualizer.Form
);
