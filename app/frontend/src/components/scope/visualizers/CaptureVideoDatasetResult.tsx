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
import { CaptureVideoDatasetResult } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";
import { CaptureVideoDatasetConfigurationElement } from "./CaptureVideoDatasetConfiguration";

export const CaptureVideoDatasetResultElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const { configuration, numFrames, stampEnd, dataset } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Num Frames", numFrames],
            ["Stamp End", stampEnd],
            ["Dataset URL", dataset?.path]
          ]}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          {
            <CaptureVideoDatasetConfigurationElement
              value={[0, configuration]}
              options={[]}
              resources={resources}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export class CaptureVideoDatasetResultVisualizer
  implements Visualizer<CaptureVideoDatasetResult> {
  static id: VisualizerId = "captureVideoDatasetResult";
  types: EventTypeId[] = [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetResult"
  ];

  options: VisualizerOptionConfig[] = [
    { label: "view", options: ["overlay", "grid"] }
  ];

  component: React.FC<VisualizerProps<CaptureVideoDatasetResult>> = (props) => {
    const view = props.options[0].value as "overlay" | "grid";
    return (
      <Layout
        view={view}
        element={CaptureVideoDatasetResultElement}
        {...props}
      />
    );
  };
}
