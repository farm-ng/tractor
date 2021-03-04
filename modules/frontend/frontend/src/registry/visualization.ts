import { TimeSkewVisualizer } from "../components/scope/visualizers/TimeSkewVisualizer";
import { JSONVisualizer } from "../components/scope/visualizers/JSONVisualizer";
import { SteeringCommandVisualizer } from "../components/scope/visualizers/SteeringCommand";
import { TimestampedEvent, TimestampedEventVector } from "../types/common";
import { EventType, EventTypeId, eventTypeIds } from "./events";
import { ResourceArchive } from "../models/ResourceArchive";
import { ImageVisualizer } from "../components/scope/visualizers/Image";
import { ApriltagDetectionsVisualizer } from "../components/scope/visualizers/ApriltagDetections";
import { NamedSE3PoseVisualizer } from "../components/scope/visualizers/NamedSE3Pose";
import { CalibrateApriltagRigStatusVisualizer } from "../components/scope/visualizers/CalibrateApriltagRigStatus";
import { CalibrateBaseToCameraStatusVisualizer } from "../components/scope/visualizers/CalibrateBaseToCameraStatus";
import { BaseToCameraModelVisualizer } from "../components/scope/visualizers/BaseToCameraModel";
import { CalibrateApriltagRigConfigurationVisualizer } from "../components/scope/visualizers/CalibrateApriltagRigConfiguration";
import { CalibrateApriltagRigResultVisualizer } from "../components/scope/visualizers/CalibrateApriltagRigResult";
import { CalibrateBaseToCameraConfigurationVisualizer } from "../components/scope/visualizers/CalibrateBaseToCameraConfiguration";
import { CalibrateBaseToCameraResultVisualizer } from "../components/scope/visualizers/CalibrateBaseToCameraResult";
import { CalibrateIntrinsicsConfigurationVisualizer } from "../components/scope/visualizers/CalibrateIntrinsicsConfiguration";
import { CalibrateIntrinsicsStatusVisualizer } from "../components/scope/visualizers/CalibrateIntrinsicsStatus";
import { CalibrateIntrinsicsResultVisualizer } from "../components/scope/visualizers/CalibrateIntrinsicsResult";
import { MonocularApriltagRigModelVisualizer } from "../components/scope/visualizers/MonocularApriltagRigModel";
import { TractorConfigVisualizer } from "../components/scope/visualizers/TractorConfig";
import { CaptureVideoDatasetConfigurationVisualizer } from "../components/scope/visualizers/CaptureVideoDatasetConfiguration";
import { CaptureVideoDatasetStatusVisualizer } from "../components/scope/visualizers/CaptureVideoDatasetStatus";
import { CaptureVideoDatasetResultVisualizer } from "../components/scope/visualizers/CaptureVideoDatasetResult";
import { CreateVideoDatasetConfigurationVisualizer } from "../components/scope/visualizers/CreateVideoDatasetConfiguration";
import { CreateVideoDatasetStatusVisualizer } from "../components/scope/visualizers/CreateVideoDatasetStatus";
import { CreateVideoDatasetResultVisualizer } from "../components/scope/visualizers/CreateVideoDatasetResult";
import { DetectApriltagsConfigurationVisualizer } from "../components/scope/visualizers/DetectApriltagsConfiguration";
import { DetectApriltagsStatusVisualizer } from "../components/scope/visualizers/DetectApriltagsStatus";
import { ApriltagConfigVisualizer } from "../components/scope/visualizers/ApriltagConfig";
import { CameraConfigVisualizer } from "../components/scope/visualizers/CameraConfig";
import { CameraPipelineConfigVisualizer } from "../components/scope/visualizers/CameraPipelineConfig";
import { CalibrateMultiViewApriltagRigConfigurationVisualizer } from "../components/scope/visualizers/CalibrateMultiViewApriltagRigConfiguration";
import { CalibrateMultiViewApriltagRigResultVisualizer } from "../components/scope/visualizers/CalibrateMultiViewApriltagRigResult";
import { CalibrateMultiViewApriltagRigStatusVisualizer } from "../components/scope/visualizers/CalibrateMultiViewApriltagRigStatus";
import { MultiViewApriltagRigModelVisualizer } from "../components/scope/visualizers/MultiViewApriltagRigModel";
import { VideoFileCameraVisualizer } from "../components/scope/visualizers/VideoFileCamera";
import { CameraModelVisualizer } from "../components/scope/visualizers/CameraModel";
import { CapturePoseRequestVisualizer } from "../components/scope/visualizers/CapturePoseRequest";
import { CapturePoseResponseVisualizer } from "../components/scope/visualizers/CapturePoseResponse";
import { CaptureRobotExtrinsicsDatasetConfigurationVisualizer } from "../components/scope/visualizers/CaptureRobotExtrinsicsDatasetConfiguration";
import { CaptureRobotExtrinsicsDatasetResultVisualizer } from "../components/scope/visualizers/CaptureRobotExtrinsicsDatasetResult";
import { CaptureRobotExtrinsicsDatasetStatusVisualizer } from "../components/scope/visualizers/CaptureRobotExtrinsicsDatasetStatus";
import { CalibrateRobotExtrinsicsConfigurationVisualizer } from "../components/scope/visualizers/CalibrateRobotExtrinsicsConfiguration";

import { MultiViewCameraRigVisualizer } from "../components/scope/visualizers/MultiViewCameraRig";
import { LogPlaybackConfigurationVisualizer } from "../components/scope/visualizers/LogPlaybackConfiguration";
import { LogPlaybackStatusVisualizer } from "../components/scope/visualizers/LogPlaybackStatus";
import { ValidateRobotExtrinsicsConfigurationVisualizer } from "../components/scope/visualizers/ValidateRobotExtrinsicsConfiguration";

export interface VisualizerOptionConfig {
  label: string;
  options: string[];
}
export type VisualizerOption = VisualizerOptionConfig & { value: string };

export interface SingleElementVisualizerProps<T extends EventType = EventType> {
  value: TimestampedEvent<T>;
  options?: VisualizerOption[];
  resources?: ResourceArchive;
}

export interface VisualizerProps<T extends EventType = EventType> {
  values: TimestampedEventVector<T>;
  options: VisualizerOption[];
  resources?: ResourceArchive;
}

export interface FormProps<T extends EventType = EventType> {
  initialValue: T;
  onChange: (updated: T) => void;
}

// [docs] Visualizer
export interface Visualizer<T extends EventType = EventType> {
  // An id, unique amongst visualizers
  id: VisualizerId;

  // Event types to visualize, or the wildcard for all types
  types: EventTypeId[] | "*";

  // Options
  options: VisualizerOptionConfig[];

  // Visualization of a single event
  Element?: React.FC<SingleElementVisualizerProps<T>>;

  // Visualization of a vector of events
  MultiElement?: React.FC<VisualizerProps<T>>;

  // A form visualization, for create/update.
  Form?: React.FC<FormProps<T>>;

  // 3D visualization of a single event
  Element3D?: React.FC<SingleElementVisualizerProps<T>>;
}
// [docs] Visualizer

export const visualizerRegistry: { [k: string]: Visualizer } = [
  ApriltagConfigVisualizer,
  ApriltagDetectionsVisualizer,
  BaseToCameraModelVisualizer,
  CalibrateApriltagRigConfigurationVisualizer,
  CalibrateApriltagRigResultVisualizer,
  CalibrateApriltagRigStatusVisualizer,
  CalibrateBaseToCameraConfigurationVisualizer,
  CalibrateBaseToCameraResultVisualizer,
  CalibrateBaseToCameraStatusVisualizer,
  CalibrateMultiViewApriltagRigConfigurationVisualizer,
  CalibrateMultiViewApriltagRigResultVisualizer,
  CalibrateMultiViewApriltagRigStatusVisualizer,
  CameraConfigVisualizer,
  CameraModelVisualizer,
  CalibrateIntrinsicsConfigurationVisualizer,
  CalibrateIntrinsicsResultVisualizer,
  CalibrateIntrinsicsStatusVisualizer,
  CapturePoseRequestVisualizer,
  CapturePoseResponseVisualizer,
  CaptureRobotExtrinsicsDatasetConfigurationVisualizer,
  CaptureRobotExtrinsicsDatasetResultVisualizer,
  CaptureRobotExtrinsicsDatasetStatusVisualizer,
  CalibrateRobotExtrinsicsConfigurationVisualizer,
  ValidateRobotExtrinsicsConfigurationVisualizer,
  CaptureVideoDatasetConfigurationVisualizer,
  CaptureVideoDatasetResultVisualizer,
  CaptureVideoDatasetStatusVisualizer,
  CreateVideoDatasetConfigurationVisualizer,
  CreateVideoDatasetResultVisualizer,
  CreateVideoDatasetStatusVisualizer,
  VideoFileCameraVisualizer,
  DetectApriltagsConfigurationVisualizer,
  DetectApriltagsStatusVisualizer,
  ImageVisualizer,
  LogPlaybackConfigurationVisualizer,
  LogPlaybackStatusVisualizer,
  MonocularApriltagRigModelVisualizer,
  MultiViewApriltagRigModelVisualizer,
  MultiViewCameraRigVisualizer,
  NamedSE3PoseVisualizer,
  SteeringCommandVisualizer,
  CameraPipelineConfigVisualizer,
  TractorConfigVisualizer,
  // Low priority, should stay at the end of the list
  JSONVisualizer,
  TimeSkewVisualizer,
].reduce((acc, visualizer) => ({ ...acc, [visualizer.id]: visualizer }), {});

export const visualizerIds = Object.keys(visualizerRegistry);
export type VisualizerId = typeof visualizerIds[number];

export function visualizersForEventType(
  eventType: EventTypeId | null
): Visualizer[] {
  return Object.entries(visualizerRegistry)
    .filter(([k, v]) => {
      // This visualizer explicitly supports this event type
      if (eventType && v.types.includes(eventType)) {
        return true;
      }
      // This visualizer supports all known event types
      if (eventType && v.types === "*" && eventTypeIds.includes(eventType)) {
        return true;
      }
      // This is an unknown event type, but at least we can visualize its timestamp
      if (k === TimeSkewVisualizer.id) {
        return true;
      }
      return false;
    })
    .map(([_, v]) => v);
}
