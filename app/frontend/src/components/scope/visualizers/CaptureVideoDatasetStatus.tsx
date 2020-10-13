/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import {
  CaptureVideoDatasetResult,
  CaptureVideoDatasetStatus
} from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  StandardComponent,
  StandardComponentOptions
} from "./StandardComponent";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";

const CaptureVideoDatasetStatusElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const result = useFetchResource<CaptureVideoDatasetResult>(
    value.result,
    resources
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
            <CaptureVideoDatasetResultVisualizer.Element
              {...props}
              value={[0, result]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureVideoDatasetStatusVisualizer = {
  id: "CaptureVideoDatasetStatus",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetStatus"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureVideoDatasetStatusElement),
  Element: CaptureVideoDatasetStatusElement
};
