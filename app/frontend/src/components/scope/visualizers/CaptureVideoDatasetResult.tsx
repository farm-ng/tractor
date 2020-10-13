/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureVideoDatasetResult } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";
import {
  StandardComponent,
  StandardComponentOptions
} from "./StandardComponent";
import { CaptureVideoDatasetConfigurationVisualizer } from "./CaptureVideoDatasetConfiguration";

const CaptureVideoDatasetResultElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetResult
>> = (props) => {
  const {
    value: [timestamp, value]
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
            <CaptureVideoDatasetConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureVideoDatasetResultVisualizer = {
  id: "CaptureVideoDatasetResult",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetResult"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureVideoDatasetResultElement),
  Element: CaptureVideoDatasetResultElement
};
