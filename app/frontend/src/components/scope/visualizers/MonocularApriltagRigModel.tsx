/* eslint-disable no-console */
import * as React from "react";
import { Table } from "react-bootstrap";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  MonocularApriltagRigModel,
  solverStatusToJSON
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrator";
import { formatValue } from "../../../utils/formatValue";
import { useState } from "react";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import RangeSlider from "react-bootstrap-range-slider";
import styles from "./MonocularApriltagRigModel.module.scss";
import { Canvas } from "../../Canvas";
import { NamedSE3Pose } from "../../../../genproto/farm_ng_proto/tractor/v1/geometry";

import { Lights } from "../../Lights";
import { Controls } from "../../Controls";
import { ReferenceFrameViz } from "../../PoseViz";
import { Ground } from "../../Ground";
import { getDagTransform } from "../../../utils/geometry";
import { KeyValueTable } from "./KeyValueTable";
import { ApriltagDetectionsVisualizer } from "./ApriltagDetections";
import { ImageVisualizer } from "./Image";

const MonocularApriltagRigModelElement: React.FC<SingleElementVisualizerProps<
  MonocularApriltagRigModel
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  const {
    detections,
    cameraFrameName,
    cameraPosesRig,
    reprojectionImages,
    tagStats,
    rig
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

  if (cameraPoseRig) {
    nodePoses.push(cameraPoseRig);
  }

  // 3D markers for each rig node
  const markers = rig?.nodes.map((entry) => {
    if (!nodes) {
      return undefined;
    }
    const aPoseB = getDagTransform(nodePoses, entry.frameName);
    if (!rigRootName || !aPoseB) {
      return undefined;
    }
    const pose: NamedSE3Pose = {
      frameA: rigRootName,
      frameB: entry.frameName,
      aPoseB
    };
    return (
      pose && (
        <ReferenceFrameViz
          key={entry.id}
          pose={pose}
          // eslint-disable-next-line react/no-children-prop
          children={[]}
        />
      )
    );
  });

  // A 3D marker for the camera_pose_rig (if it exists)
  if (cameraPoseRig && rigRootName && markers) {
    const cameraTransform = getDagTransform(nodePoses, cameraPoseRig.frameA);
    markers.push(
      <ReferenceFrameViz
        key={rigRootName}
        pose={{
          frameA: rigRootName,
          frameB: cameraPoseRig.frameA,
          aPoseB: cameraTransform || undefined
        }}
        // eslint-disable-next-line react/no-children-prop
        children={[]}
      />
    );
  }

  return (
    <Card json={value} timestamp={timestamp}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatusToJSON(value.solverStatus)],
            ["Total RMSE", value.rmse]
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
          <Canvas>
            <Lights />
            <Ground />
            <fogExp2 args={[0xcccccc, 0.02]} />
            <Controls />
            {markers}
          </Canvas>
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
          <Card className={styles.card}>
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
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.MonocularApriltagRigModel"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(MonocularApriltagRigModelElement),
  Element: MonocularApriltagRigModelElement
};
