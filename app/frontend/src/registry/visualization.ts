import { TimeSkewVisualizer } from "../components/scope/visualizers/TimeSkewVisualizer";
import { JSONVisualizer } from "../components/scope/visualizers/JSONVisualizer";
import { SteeringCommandVisualizer } from "../components/scope/visualizers/SteeringCommandVisualizer";
import { TimestampedEvent, TimestampedEventVector } from "../types/common";
import { EventType, EventTypeId, eventTypeIds } from "./events";
import { ResourceArchive } from "../models/ResourceArchive";
import { ImageVisualizer } from "../components/scope/visualizers/ImageVisualizer";
import { ApriltagDetectionsVisualizer } from "../components/scope/visualizers/ApriltagDetectionsVisualizer";
import { NamedSE3PoseVisualizer } from "../components/scope/visualizers/NamedSE3PoseVisualizer";
import { TrackingCameraPoseFrameVisualizer } from "../components/scope/visualizers/TrackingCameraPoseFrameVisualizer";
import { CalibrateApriltagRigStatusVisualizer } from "../components/scope/visualizers/CalibrateApriltagRigStatus";
import { CalibrateBaseToCameraStatusVisualizer } from "../components/scope/visualizers/CalibrateBaseToCameraStatus";
import { BaseToCameraModelVisualizer } from "../components/scope/visualizers/BaseToCameraModel";
import { CalibrateApriltagRigConfigurationVisualizer } from "../components/scope/visualizers/CalibrateApriltagRigConfiguration";
import { CalibrateApriltagRigResultVisualizer } from "../components/scope/visualizers/CalibrateApriltagRigResult";
import { CalibrateBaseToCameraConfigurationVisualizer } from "../components/scope/visualizers/CalibrateBaseToCameraConfiguration";
import { CalibrateBaseToCameraResultVisualizer } from "../components/scope/visualizers/CalibrateBaseToCameraResult";
import { CaptureCalibrationDatasetConfigurationVisualizer } from "../components/scope/visualizers/CaptureCalibrationDatasetConfiguration";
import { CaptureCalibrationDatasetStatusVisualizer } from "../components/scope/visualizers/CaptureCalibrationDatasetStatus";
import { CaptureCalibrationDatasetResultVisualizer } from "../components/scope/visualizers/CaptureCalibrationDatasetResult";
import { MonocularApriltagRigModelVisualizer } from "../components/scope/visualizers/MonocularApriltagRigModel";
import { CaptureVideoDatasetConfigurationVisualizer } from "../components/scope/visualizers/CaptureVideoDatasetConfiguration";
import { CaptureVideoDatasetResultVisualizer } from "../components/scope/visualizers/CaptureVideoDatasetResult";
import { CaptureVideoDatasetStatusVisualizer } from "../components/scope/visualizers/CaptureVideoDatasetStatus";

export interface VisualizerOptionConfig {
  label: string;
  options: string[];
}
export type VisualizerOption = VisualizerOptionConfig & { value: string };

export interface SingleElementVisualizerProps<T extends EventType = EventType> {
  value: TimestampedEvent<T>;
  options: VisualizerOption[];
  resources: ResourceArchive | null;
}

export interface VisualizerProps<T extends EventType = EventType> {
  values: TimestampedEventVector<T>;
  options: VisualizerOption[];
  resources: ResourceArchive | null;
}

export interface Visualizer<T extends EventType = EventType> {
  component: React.FC<VisualizerProps<T>>;
  options: VisualizerOptionConfig[];
  types: EventTypeId[] | "*";
}

export const visualizerRegistry: { [k: string]: Visualizer } = {
  [ApriltagDetectionsVisualizer.id]: new ApriltagDetectionsVisualizer() as Visualizer,
  [BaseToCameraModelVisualizer.id]: new BaseToCameraModelVisualizer() as Visualizer,
  [CalibrateApriltagRigConfigurationVisualizer.id]: new CalibrateApriltagRigConfigurationVisualizer() as Visualizer,
  [CalibrateApriltagRigResultVisualizer.id]: new CalibrateApriltagRigResultVisualizer() as Visualizer,
  [CalibrateApriltagRigStatusVisualizer.id]: new CalibrateApriltagRigStatusVisualizer() as Visualizer,
  [CalibrateBaseToCameraConfigurationVisualizer.id]: new CalibrateBaseToCameraConfigurationVisualizer() as Visualizer,
  [CalibrateBaseToCameraResultVisualizer.id]: new CalibrateBaseToCameraResultVisualizer() as Visualizer,
  [CalibrateBaseToCameraStatusVisualizer.id]: new CalibrateBaseToCameraStatusVisualizer() as Visualizer,
  [CaptureCalibrationDatasetConfigurationVisualizer.id]: new CaptureCalibrationDatasetConfigurationVisualizer() as Visualizer,
  [CaptureCalibrationDatasetResultVisualizer.id]: new CaptureCalibrationDatasetResultVisualizer() as Visualizer,
  [CaptureCalibrationDatasetStatusVisualizer.id]: new CaptureCalibrationDatasetStatusVisualizer() as Visualizer,
  [CaptureVideoDatasetConfigurationVisualizer.id]: new CaptureVideoDatasetConfigurationVisualizer() as Visualizer,
  [CaptureVideoDatasetResultVisualizer.id]: new CaptureVideoDatasetResultVisualizer() as Visualizer,
  [CaptureVideoDatasetStatusVisualizer.id]: new CaptureVideoDatasetStatusVisualizer() as Visualizer,
  [ImageVisualizer.id]: new ImageVisualizer() as Visualizer,
  [TrackingCameraPoseFrameVisualizer.id]: new TrackingCameraPoseFrameVisualizer() as Visualizer,
  [MonocularApriltagRigModelVisualizer.id]: new MonocularApriltagRigModelVisualizer() as Visualizer,
  [NamedSE3PoseVisualizer.id]: new NamedSE3PoseVisualizer() as Visualizer,
  [SteeringCommandVisualizer.id]: new SteeringCommandVisualizer() as Visualizer,
  [TrackingCameraPoseFrameVisualizer.id]: new TrackingCameraPoseFrameVisualizer() as Visualizer,
  [JSONVisualizer.id]: new JSONVisualizer() as Visualizer,
  [TimeSkewVisualizer.id]: new TimeSkewVisualizer() as Visualizer
};
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
