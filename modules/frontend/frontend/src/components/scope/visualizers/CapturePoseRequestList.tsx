/* eslint-disable no-console */
import { CapturePoseRequest } from "@farm-ng/genproto-calibration/farm_ng/calibration/robot_hal";
import { NamedSE3Pose } from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { JointStatePlot } from "./JointStatePlot";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";
import { Scene } from "./Scene";
import styles from "./CapturePoseRequestList.module.scss";

// Visualizes a list of CapturePoseRequests
export const CapturePoseRequestList: React.FC<{
  requests: CapturePoseRequest[];
  // Optionally visualize which requests have completed
  latestRequestIndex?: number;
}> = ({ requests, latestRequestIndex }) => {
  const poses = requests.reduce<
    ReturnType<React.FC<SingleElementVisualizerProps<NamedSE3Pose>>>[]
  >((acc, req, index) => {
    req.poses.forEach((pose) => {
      // If latestRequestIndex is provided, mark completed poses with an asterisk
      const suffix =
        latestRequestIndex !== undefined && index <= latestRequestIndex
          ? "*"
          : "";
      acc.push(
        <NamedSE3PoseVisualizer.Element3D
          key={`${pose.frameA}:${pose.frameB}:${index}`}
          showFrameA={index === 0}
          value={[
            0,
            {
              ...pose,
              frameB: `${pose.frameB}_${index}${suffix}`,
            },
          ]}
        />
      );
    });
    return acc;
  }, []);

  let jointStates = requests.map((_) => _.jointStates);

  // If latestRequestIndex is provided, mark completed requests with an asterisk
  const suffix = latestRequestIndex !== undefined ? "(* complete)" : "";

  return (
    <div className={styles.scenePair}>
      <div>
        <h6>{`Pose Requests ${suffix}`}</h6>
        <Scene groundTransparency>{poses}</Scene>
      </div>
      {jointStates && (
        <div>
          <h6>{`Joint State Requests ${suffix}`}</h6>
          <JointStatePlot values={jointStates} completed={latestRequestIndex} />
        </div>
      )}
    </div>
  );
};
