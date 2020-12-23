/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { CapturePoseRequest } from "@farm-ng/genproto-calibration/farm_ng/calibration/robot_hal";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { Card } from "./Card";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";
import { Scene } from "./Scene";

const CapturePoseRequestElement: React.FC<SingleElementVisualizerProps<
  CapturePoseRequest
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const poses = value.poses.map((pose, index) => {
    return (
      <NamedSE3PoseVisualizer.Marker3D
        key={`${pose.frameA}:${pose.frameB}:${index}`}
        value={[0, pose]}
      />
    );
  });

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <Scene groundTransparency>{poses}</Scene>
      </Card>
    </Card>
  );
};

export const CapturePoseRequestVisualizer = {
  id: "CapturePoseRequest",
  types: ["type.googleapis.com/farm_ng.calibration.CapturePoseRequest"],
  options: StandardComponentOptions,
  Component: StandardComponent(CapturePoseRequestElement),
  Element: CapturePoseRequestElement,
};