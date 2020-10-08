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
import {
  CaptureVideoDatasetResult,
  CaptureVideoDatasetStatus
} from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { CaptureVideoDatasetResultElement } from "./CaptureVideoDatasetResult";

export const CaptureVideoDatasetStatusElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const result = useFetchResource<CaptureVideoDatasetResult>(
    value.result,
    resources || undefined
  );
  const { numFrames } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable records={[["Num Frames", numFrames]]} />
      </Card>
      {result && (
        <Card title="Result">
          {
            <CaptureVideoDatasetResultElement
              value={[0, result]}
              options={[]}
              resources={resources}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export class CaptureVideoDatasetStatusVisualizer
  implements Visualizer<CaptureVideoDatasetStatus> {
  static id: VisualizerId = "captureVideoDatasetStatus";
  types: EventTypeId[] = [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetStatus"
  ];

  options: VisualizerOptionConfig[] = [
    { label: "view", options: ["overlay", "grid"] }
  ];

  component: React.FC<VisualizerProps<CaptureVideoDatasetStatus>> = (props) => {
    const view = props.options[0].value as "overlay" | "grid";
    return (
      <Layout
        view={view}
        element={CaptureVideoDatasetStatusElement}
        {...props}
      />
    );
  };
}
