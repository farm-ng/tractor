/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureCalibrationDatasetResult } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import { CaptureCalibrationDatasetConfigurationVisualizer } from "./CaptureCalibrationDatasetConfiguration";

const CaptureCalibrationDatasetResultElement: React.FC<SingleElementVisualizerProps<
  CaptureCalibrationDatasetResult
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  const { configuration, numFrames, tagIds, stampEnd, dataset } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Num Frames", numFrames],
            ["Tag IDs", (tagIds || []).join(", ")],
            ["Stamp End", stampEnd],
            ["Dataset URL", dataset?.path]
          ]}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          {
            <CaptureCalibrationDatasetConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureCalibrationDatasetResultVisualizer = {
  id: "CaptureCalibrationDatasetResult",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetResult"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureCalibrationDatasetResultElement),
  Element: CaptureCalibrationDatasetResultElement
};
