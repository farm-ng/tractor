/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  CapturePoseResponse,
  capturePoseResponse_StatusToJSON,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/robot_hal";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { Card } from "./Card";
import { KeyValueTable } from "./KeyValueTable";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";
import { Scene } from "./Scene";
import { ImageVisualizer } from "./Image";
import { useStores } from "../../../hooks/useStores";
import styles from "./CapturePoseResponse.module.scss";
import { jointState_UnitsToJSON } from "@farm-ng/genproto-perception/farm_ng/perception/kinematics";

const CapturePoseResponseElement: React.FC<SingleElementVisualizerProps<
  CapturePoseResponse
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const { status, images, jointStates } = value;
  const { httpResourceArchive } = useStores();

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
      <Card title="Summary">
        <KeyValueTable
          records={[["Status", capturePoseResponse_StatusToJSON(status)]]}
        />
      </Card>
      <Card title="RGB Images">
        <div className={styles.imageRow}>
          {images.map((image, index) => (
            <ImageVisualizer.Element
              key={index}
              value={[timestamp, image]}
              resources={httpResourceArchive}
            />
          ))}
        </div>
      </Card>
      <Card title="Depth Images">
        <div className={[styles.imageRow, styles.depthImages].join(" ")}>
          {images.map((image, index) => (
            <ImageVisualizer.Element
              key={index}
              value={[timestamp, image]}
              depth
              resources={httpResourceArchive}
            />
          ))}
        </div>
      </Card>
      {(poses.length > 0 || jointStates.length > 0) && (
        <Card title="Achieved Poses and Joint States">
          <div className={styles.scenePair}>
            <Scene groundTransparency>{poses}</Scene>
            <KeyValueTable
              headers={["Joint Name", "Value", "Units"]}
              records={jointStates.map((_) => [
                _.name,
                _.value,
                jointState_UnitsToJSON(_.units),
              ])}
            />
          </div>
        </Card>
      )}
    </Card>
  );
};

export const CapturePoseResponseVisualizer = {
  id: "CapturePoseResponse",
  types: ["type.googleapis.com/farm_ng.calibration.CapturePoseResponse"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CapturePoseResponseElement),
  Element: CapturePoseResponseElement,
};
