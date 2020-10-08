/* eslint-disable no-console */
import * as React from "react";
import {
  SingleElementVisualizerProps,
  Visualizer,
  VisualizerId,
  VisualizerOptionConfig,
  VisualizerProps
} from "../../../registry/visualization";
import { EventTypeId } from "../../../registry/events";
import { Layout } from "./Layout";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureVideoDatasetConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";

export const CaptureVideoDatasetConfigurationElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetConfiguration
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  const { name } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable records={[["Name", name]]} />
      </Card>
    </Card>
  );
};

export class CaptureVideoDatasetConfigurationVisualizer
  implements Visualizer<CaptureVideoDatasetConfiguration> {
  static id: VisualizerId = "captureVideoDatasetConfiguration";
  types: EventTypeId[] = [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetConfiguration"
  ];

  options: VisualizerOptionConfig[] = [
    { label: "view", options: ["overlay", "grid"] }
  ];

  component: React.FC<VisualizerProps<CaptureVideoDatasetConfiguration>> = (
    props
  ) => {
    const view = props.options[0].value as "overlay" | "grid";
    return (
      <Layout
        view={view}
        element={CaptureVideoDatasetConfigurationElement}
        {...props}
      />
    );
  };
}
