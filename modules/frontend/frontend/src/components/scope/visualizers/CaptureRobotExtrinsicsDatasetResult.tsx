/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureRobotExtrinsicsDatasetResult } from "@farm-ng/genproto-calibration/farm_ng/calibration/capture_robot_extrinsics_dataset";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { CaptureRobotExtrinsicsDatasetConfigurationVisualizer } from "./CaptureRobotExtrinsicsDatasetConfiguration";

const CaptureRobotExtrinsicsDatasetResultElement: React.FC<SingleElementVisualizerProps<
  CaptureRobotExtrinsicsDatasetResult
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const { configuration, stampBegin, stampEnd, dataset } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Stamp Begin", stampBegin],
            ["Stamp End", stampEnd],
            ["Dataset URL", dataset?.path],
          ]}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          {
            <CaptureRobotExtrinsicsDatasetConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureRobotExtrinsicsDatasetResultVisualizer = {
  id: "CaptureRobotExtrinsicsDatasetResult",
  types: [
    "type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetResult",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    CaptureRobotExtrinsicsDatasetResultElement
  ),
  Element: CaptureRobotExtrinsicsDatasetResultElement,
};
