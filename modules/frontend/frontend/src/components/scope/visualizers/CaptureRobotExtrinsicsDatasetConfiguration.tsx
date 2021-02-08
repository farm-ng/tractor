/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureRobotExtrinsicsDatasetConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/capture_robot_extrinsics_dataset";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { ApriltagRigVisualizer } from "./ApriltagRig";
import { Scene } from "./Scene";
import { MultiViewCameraRigVisualizer } from "./MultiViewCameraRig";
import { CapturePoseRequestList } from "./CapturePoseRequestList";
import styles from "./CaptureRobotExtrinsicsDatasetConfiguration.module.scss";

const CaptureRobotExtrinsicsDatasetConfigurationElement: React.FC<SingleElementVisualizerProps<
  CaptureRobotExtrinsicsDatasetConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const {
    name,
    workspaceFrameName,
    baseFrameName,
    linkFrameName,
    halServiceAddress,
  } = value;

  const baseCameraRig = value.baseCameraRig && (
    <MultiViewCameraRigVisualizer.Element3D value={[0, value.baseCameraRig]} />
  );

  const baseTagRig = value.baseTagRig && (
    <ApriltagRigVisualizer.Element3D value={[0, value.baseTagRig]} />
  );

  const linkCameraRig = value.linkCameraRig && (
    <MultiViewCameraRigVisualizer.Element3D value={[0, value.linkCameraRig]} />
  );

  const linkTagRig = value.linkTagRig && (
    <ApriltagRigVisualizer.Element3D value={[0, value.linkTagRig]} />
  );

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Name", name],
            ["HAL Service Address", halServiceAddress],
            ["Workspace Frame Name", workspaceFrameName],
            ["Base Frame Name", baseFrameName],
            ["Link Frame Name", linkFrameName],
          ]}
        />
      </Card>
      <Card title="Request Queue">
        {value.requestQueue && (
          <CapturePoseRequestList requests={value.requestQueue} />
        )}
      </Card>
      <Card title="Rigs">
        <div className={styles.scenePair}>
          <div>
            <h6>Base Camera Rig</h6>
            <Scene groundTransparency>{baseCameraRig}</Scene>
          </div>
          <div>
            <h6>Base Tag Rig</h6>
            <Scene groundTransparency>{baseTagRig}</Scene>
          </div>
        </div>
        <div className={styles.scenePair}>
          <div>
            <h6>Link Camera Rig</h6>
            <Scene groundTransparency>{linkCameraRig}</Scene>
          </div>
          <div>
            <h6>Link Tag Rig</h6>
            <Scene groundTransparency>{linkTagRig}</Scene>
          </div>
        </div>
      </Card>
    </Card>
  );
};

export const CaptureRobotExtrinsicsDatasetConfigurationVisualizer = {
  id: "CaptureRobotExtrinsicsDatasetConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetConfiguration",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    CaptureRobotExtrinsicsDatasetConfigurationElement
  ),
  Element: CaptureRobotExtrinsicsDatasetConfigurationElement,
};
