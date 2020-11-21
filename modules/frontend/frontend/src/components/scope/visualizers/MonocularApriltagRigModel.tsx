/* eslint-disable no-console */
import * as React from "react";
import { Table } from "react-bootstrap";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  MonocularApriltagRigModel,
  solverStatusToJSON,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { formatValue } from "../../../utils/formatValue";
import { cloneElement, useState } from "react";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import RangeSlider from "react-bootstrap-range-slider";
import styles from "./MonocularApriltagRigModel.module.scss";
import { Scene } from "./Scene";
import { NamedSE3Pose } from "@farm-ng/genproto-perception/farm_ng/perception/geometry";

import { getDagTransform } from "../../../utils/geometry";
import { KeyValueTable } from "./KeyValueTable";
import { ApriltagDetectionsVisualizer } from "./ApriltagDetections";
import { ImageVisualizer } from "./Image";
import { PerspectiveCamera } from "./Camera";
import { toQuaternion, toVector3 } from "../../../utils/protoConversions";
import { Euler, Quaternion } from "three";
import { ApriltagRigVisualizer } from "./ApriltagRig";
import { FisheyeEffect } from "./FisheyeEffect";
import { Sky } from "drei";

const MonocularApriltagRigModelElement: React.FC<SingleElementVisualizerProps<
  MonocularApriltagRigModel
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const {
    detections,
    cameraFrameName,
    cameraPosesRig,
    reprojectionImages,
    tagStats,
    rig,
  } = value || {};

  // The currently selected image index
  const [index, setIndex] = useState(0);
  const detection = detections?.[index];

  // The index of the camera_pose_rig estimation (if one exists for the currently selected image index)
  const cameraPoseRigIndex = (cameraPosesRig || []).findIndex((pose) => {
    const { frameA } = pose;
    const [, frameIndex] = frameA.split(cameraFrameName + "/");
    return parseInt(frameIndex) === index;
  });
  const cameraPoseRig = cameraPosesRig?.[cameraPoseRigIndex];
  const reprojectionImage = reprojectionImages?.[cameraPoseRigIndex];

  // Per-tag RMSE, by tag ID, for the currently selected image index
  const tagRmses = (tagStats || []).reduce<{ [key: number]: number }>(
    (acc, tagStat) => {
      const rmseEntry = tagStat.perImageRmse.find(
        (_) => _.frameNumber === index
      );
      if (rmseEntry) {
        acc[tagStat.tagId] = rmseEntry.rmse;
      }
      return acc;
    },
    {}
  );

  // Collect rig nodes, plus the camera_pose_rig for the current image index
  const nodes = rig?.nodes;
  const rigRootName = nodes?.find((_) => _.id === rig?.rootTagId)?.frameName;
  const nodePoses = (nodes || [])
    .sort((a, b) =>
      a.id === rig?.rootTagId ? -1 : b.id === rig?.rootTagId ? 1 : 0
    )
    .map((_) => _.pose) as NamedSE3Pose[];

  const markers = rig && <ApriltagRigVisualizer.Marker3D value={[0, rig]} />;

  if (cameraPoseRig) {
    nodePoses.push(cameraPoseRig);
  }

  // A 3D marker for the camera_pose_rig (if it exists)
  let camera = null;
  let defaultCamera = null;
  if (cameraPoseRig && rigRootName) {
    const cameraTransform = getDagTransform(nodePoses, cameraPoseRig.frameA);
    const quaternion = toQuaternion(cameraTransform?.rotation);
    const opencvTthreejs = new Quaternion().setFromEuler(
      new Euler(Math.PI, 0, 0)
    );

    quaternion.multiply(opencvTthreejs);

    camera = (
      <PerspectiveCamera
        showHelper
        fov={80}
        zoom={0.5}
        position={toVector3(cameraTransform?.position)}
        quaternion={quaternion}
      />
    );
    defaultCamera = cloneElement(camera, {
      makeDefault: true,
      showHelper: false,
    });
  }

  return (
    <Card json={value} timestamp={timestamp}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatusToJSON(value.solverStatus)],
            ["Total RMSE", value.rmse],
          ]}
        />
      </Card>

      {tagStats && (
        <Card title="Tag Statistics">
          <Table striped bordered size="sm" responsive="md">
            <thead>
              <tr>
                <th>Tag Id</th>
                <th># Frames</th>
                <th>RMSE</th>
              </tr>
            </thead>
            <tbody>
              {tagStats.map((tagStat) => (
                <tr key={tagStat.tagId}>
                  <td>{tagStat.tagId}</td>
                  <td>{tagStat.nFrames}</td>
                  <td>{formatValue(tagStat.tagRigRmse)}</td>
                </tr>
              ))}
            </tbody>
          </Table>
        </Card>
      )}

      {markers && (
        <Card title="Apriltag Rig">
          <div className={styles.scenePair}>
            <Card>
              <Scene groundTransparency={true}>
                {markers}
                {camera}
              </Scene>
            </Card>
            <Card>
              <Scene controls={false} ground={false}>
                <Sky />
                <FisheyeEffect />
                {markers}
                {defaultCamera}
              </Scene>
            </Card>
          </div>
        </Card>
      )}

      {(detection || reprojectionImage || tagRmses) && (
        <Card title="Details">
          <RangeSlider
            value={index}
            onChange={(_, v) => setIndex(v)}
            min={0}
            max={value.detections.length ? value.detections.length - 1 : 0}
            step={1}
          />
          <div className={styles.detailRow}>
            {detection && (
              // Unfortunately we don't have the timestamp for these detections
              <ApriltagDetectionsVisualizer.Element
                {...props}
                value={[0, detection]}
              />
            )}
            {reprojectionImage && (
              <ImageVisualizer.Element
                {...props}
                value={[0, reprojectionImage]}
              />
            )}
          </div>
          <Card>
            {Object.keys(tagRmses).length > 0 && (
              <Table striped bordered size="sm" responsive="md">
                <thead>
                  <tr>
                    <th>Tag Id</th>
                    <th>RMSE</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.entries(tagRmses).map(([tagId, rmse]) => (
                    <tr key={tagId}>
                      <td>{tagId}</td>
                      <td>{formatValue(rmse)}</td>
                    </tr>
                  ))}
                </tbody>
              </Table>
            )}
          </Card>
        </Card>
      )}
    </Card>
  );
};

export const MonocularApriltagRigModelVisualizer = {
  id: "MonocularApriltagRigModel",
  types: ["type.googleapis.com/farm_ng.calibration.MonocularApriltagRigModel"],
  options: StandardComponentOptions,
  Component: StandardComponent(MonocularApriltagRigModelElement),
  Element: MonocularApriltagRigModelElement,
};
