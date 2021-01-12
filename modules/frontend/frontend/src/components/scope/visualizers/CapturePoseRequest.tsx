/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { CapturePoseRequest } from "@farm-ng/genproto-calibration/farm_ng/calibration/robot_hal";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { Card } from "./Card";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";
import { Scene } from "./Scene";
import { jointState_UnitsToJSON } from "@farm-ng/genproto-perception/farm_ng/perception/kinematics";
import { KeyValueTable } from "./KeyValueTable";
import styles from "./CapturePoseRequest.module.scss";

const CapturePoseRequestElement: React.FC<SingleElementVisualizerProps<
  CapturePoseRequest
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const poses = value.poses.map((pose, index) => {
    return (
      <NamedSE3PoseVisualizer.Element3D
        key={`${pose.frameA}:${pose.frameB}:${index}`}
        showFrameA={index === 0}
        value={[0, pose]}
      />
    );
  });

  return (
    <Card timestamp={timestamp} json={value}>
      <div className={styles.scenePair}>
        <Scene groundTransparency>{poses}</Scene>
        <KeyValueTable
          headers={["Joint Name", "Value", "Units"]}
          records={value.jointStates.map((_) => [
            _.name,
            _.value,
            jointState_UnitsToJSON(_.units),
          ])}
        />
      </div>
    </Card>
  );
};

export const CapturePoseRequestVisualizer = {
  id: "CapturePoseRequest",
  types: ["type.googleapis.com/farm_ng.calibration.CapturePoseRequest"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CapturePoseRequestElement),
  Element: CapturePoseRequestElement,
};
