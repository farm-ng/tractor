/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureRobotExtrinsicsDatasetConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/capture_robot_extrinsics_dataset";
import {
  StandardComponent,
  StandardComponentOptions,
} from "./StandardComponent";
import { ApriltagRigVisualizer } from "./ApriltagRig";
import { Scene } from "./Scene";
import { MultiViewCameraRigVisualizer } from "./MultiViewCameraRig";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";

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
    <MultiViewCameraRigVisualizer.Marker3D value={[0, value.baseCameraRig]} />
  );

  const baseTagRig = value.baseTagRig && (
    <ApriltagRigVisualizer.Marker3D value={[0, value.baseTagRig]} />
  );

  const linkCameraRig = value.linkCameraRig && (
    <MultiViewCameraRigVisualizer.Marker3D value={[0, value.linkCameraRig]} />
  );

  const linkTagRig = value.linkTagRig && (
    <ApriltagRigVisualizer.Marker3D value={[0, value.linkTagRig]} />
  );

  const requestPoses = value.requestQueue.map((req, index) => {
    // TODO: Support chains of poses
    const pose = req.poses[0];
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
      <Card title="Request Poses">
        <Scene groundTransparency>{requestPoses}</Scene>
      </Card>
      <Card title="Base Camera Rig">
        <Scene groundTransparency>{baseCameraRig}</Scene>
      </Card>
      <Card title="Base Tag Rig">
        <Scene groundTransparency>{baseTagRig}</Scene>
      </Card>
      <Card title="Link Camera Rig">
        <Scene groundTransparency>{linkCameraRig}</Scene>
      </Card>
      <Card title="Link Tag Rig">
        <Scene groundTransparency>{linkTagRig}</Scene>
      </Card>
    </Card>
  );
};

export const CaptureRobotExtrinsicsDatasetConfigurationVisualizer = {
  id: "CaptureRobotExtrinsicsDatasetConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetConfiguration",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(
    CaptureRobotExtrinsicsDatasetConfigurationElement
  ),
  Element: CaptureRobotExtrinsicsDatasetConfigurationElement,
};