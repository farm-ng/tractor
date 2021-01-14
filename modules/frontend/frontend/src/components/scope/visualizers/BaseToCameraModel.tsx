/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  BaseToCameraModel,
  solverStatusToJSON,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { Card } from "./Card";
import { KeyValueTable } from "./KeyValueTable";
import { BaseToCameraInitializationVisualizer } from "./BaseToCameraInitialization";

const BaseToCameraModelElement: React.FC<SingleElementVisualizerProps<
  BaseToCameraModel
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const {
    solverStatus,
    rmse,
    wheelBaseline,
    wheelRadius,
    basePoseCamera,
    // samples,
    initialization,
  } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatus && solverStatusToJSON(solverStatus)],
            ["Total RMSE", rmse],
            ["Wheel Baseline", wheelBaseline],
            ["Wheel Radius", wheelRadius],
            ["base_pose_camera.x", basePoseCamera?.aPoseB?.position?.x],
            ["base_pose_camera.y", basePoseCamera?.aPoseB?.position?.y],
            ["base_pose_camera.z", basePoseCamera?.aPoseB?.position?.z],
          ]}
        />
      </Card>
      {initialization && (
        <Card title="Initialization">
          <BaseToCameraInitializationVisualizer.Element
            value={[0, initialization]}
          />
        </Card>
      )}
    </Card>
  );
};

export const BaseToCameraModelVisualizer = {
  id: "BaseToCameraModel",
  types: ["type.googleapis.com/farm_ng.calibration.BaseToCameraModel"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(BaseToCameraModelElement),
  Element: BaseToCameraModelElement,
};
