/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import {
  CaptureCalibrationDatasetResult,
  CaptureCalibrationDatasetStatus
} from "../../../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { CaptureCalibrationDatasetResultVisualizer } from "./CaptureCalibrationDatasetResult";

const CaptureCalibrationDatasetStatusElement: React.FC<SingleElementVisualizerProps<
  CaptureCalibrationDatasetStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const result = useFetchResource<CaptureCalibrationDatasetResult>(
    value.result,
    resources
  );
  const { numFrames, tagIds } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Num Frames", numFrames],
            ["Tag IDs", (tagIds || []).join(", ")]
          ]}
        />
      </Card>
      {result && (
        <Card title="Result">
          {
            <CaptureCalibrationDatasetResultVisualizer.Element
              {...props}
              value={[0, result]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureCalibrationDatasetStatusVisualizer = {
  id: "CaptureCalibrationDatasetStatus",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetStatus"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureCalibrationDatasetStatusElement),
  Element: CaptureCalibrationDatasetStatusElement
};
