/* eslint-disable no-console */
import * as React from "react";
import { Table } from "react-bootstrap";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  MultiViewApriltagRigModel,
  solverStatusToJSON,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { formatValue } from "../../../utils/formatValue";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { Scene } from "./Scene";

import { KeyValueTable } from "./KeyValueTable";
import { PerspectiveCamera } from "./Camera";
import {
  cameraModelToThreeJSFOV,
  matrix4ToSE3Pose,
  openCVPoseToThreeJSPose,
  toQuaternion,
  toVector3,
} from "../../../utils/protoConversions";
import { Matrix4 } from "three";
import { ApriltagRigVisualizer } from "./ApriltagRig";
import { useState } from "react";
import RangeSlider from "react-bootstrap-range-slider";
import { getInverse } from "../../../utils/geometry";
import { ImageVisualizer } from "./Image";
import { ApriltagDetectionsVisualizer } from "./ApriltagDetections";
import styles from "./MultiViewApriltagRigModel.module.scss";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";

const MultiViewApriltagRigModelElement: React.FC<SingleElementVisualizerProps<
  MultiViewApriltagRigModel
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const { multiViewDetections, tagStats, cameraRigPosesApriltagRig } = value;

  // The currently selected image index
  const [index, setIndex] = useState(0);
  const maxImageIndex = cameraRigPosesApriltagRig.length
    ? cameraRigPosesApriltagRig.length - 1
    : 0;

  // The current selected view index
  const [viewIndex, setViewIndex] = useState(0);
  const maxViewIndex = value.cameraRig?.cameras.length
    ? value.cameraRig.cameras.length - 1
    : 0;

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

  // Apriltag Rig Visualization
  const apriltagRig = value.apriltagRig && (
    <ApriltagRigVisualizer.Marker3D value={[0, value.apriltagRig]} />
  );

  // Camera Rig Visualization
  // TODO: Don't assume pose.frameA is the root of the camera rig
  const cameras = value.cameraRig?.cameraPoseRig.map((pose) => {
    const camera = value.cameraRig?.cameras.find(
      (c) => c.frameName === pose.frameB
    );
    return {
      fov: camera ? cameraModelToThreeJSFOV(camera) : 80,
      aspect: camera ? camera.imageWidth / camera.imageHeight : 1,
      pose: openCVPoseToThreeJSPose(pose),
    };
  });

  const cameraRig = cameras?.map((camera) => {
    const key = `${camera.pose.frameA}:${camera.pose.frameB}`;
    return (
      <NamedSE3PoseVisualizer.Marker3D key={key} value={[0, camera.pose]}>
        <PerspectiveCamera
          showHelper
          fov={camera.fov}
          far={0.5}
          aspect={camera.aspect}
        />
      </NamedSE3PoseVisualizer.Marker3D>
    );
  });

  // First-person View
  // TODO(isherman): Enable
  // const firstPersonCamera = cameras && cameras[viewIndex] && (
  //   <PerspectiveCamera
  //     makeDefault
  //     fov={cameras[viewIndex].fov}
  //     far={1000}
  //     aspect={cameras[viewIndex].aspect}
  //     zoom={0.5}
  //     position={toVector3(cameras[viewIndex].pose?.aPoseB?.position)}
  //     quaternion={toQuaternion(cameras[viewIndex].pose?.aPoseB?.rotation)}
  //   />
  // );
  // const firstPersonScene = (
  //   <Scene controls={false} ground={false}>
  //     <Sky />
  //     <FisheyeEffect />
  //     {apriltagRig}
  //     <group
  //       position={toVector3(apriltagRigPoseCameraRig.position)}
  //       quaternion={toQuaternion(apriltagRigPoseCameraRig.rotation)}
  //     >
  //       {firstPersonCamera}
  //     </group>
  //   </Scene>
  // );

  const apriltagRigPoseCameraRig = getInverse(
    cameraRigPosesApriltagRig[index].aPoseB || matrix4ToSE3Pose(new Matrix4())
  );

  // Apriltag Detections
  const apriltagDetections =
    multiViewDetections[viewIndex].detectionsPerView[index];

  // Reprojection Images
  const reprojectionImage = value.reprojectionImages[index];

  return (
    <Card json={value} timestamp={timestamp}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatusToJSON(value.solverStatus)],
            ["Total RMSE", value.rmse],
          ]}
        />
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

      <Card title="Details">
        <h6> Image Index </h6>
        <RangeSlider
          value={index}
          onChange={(_, v) => setIndex(v)}
          min={0}
          max={maxImageIndex}
          step={1}
        />
        <div className={styles.sceneTablePair}>
          <Card title={"Apriltag Rig + Camera Rig"}>
            <Scene groundTransparency={true}>
              {apriltagRig}
              <group
                position={toVector3(apriltagRigPoseCameraRig.position)}
                quaternion={toQuaternion(apriltagRigPoseCameraRig.rotation)}
              >
                {cameraRig}
              </group>
            </Scene>
          </Card>
          <Card title={"Tag Details"}>
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
          </Card>
        </div>
        <Card title="Reprojection Images">
          {reprojectionImage && (
            <ImageVisualizer.Element
              {...props}
              value={[0, reprojectionImage]}
            />
          )}
        </Card>

        <Card title={"Apriltag Detections"}>
          <h6> View </h6>
          <RangeSlider
            value={viewIndex}
            onChange={(_, v) => setViewIndex(v)}
            min={0}
            max={maxViewIndex}
            step={1}
            tooltipLabel={() =>
              value.cameraRig?.cameras[viewIndex].frameName || viewIndex
            }
          />

          {/* Unfortunately we don't have the timestamp for these detections */}
          {apriltagDetections && (
            <ApriltagDetectionsVisualizer.Element
              {...props}
              value={[0, apriltagDetections]}
            />
          )}
        </Card>
      </Card>
    </Card>
  );
};

export const MultiViewApriltagRigModelVisualizer = {
  id: "MultiViewApriltagRigModel",
  types: ["type.googleapis.com/farm_ng.calibration.MultiViewApriltagRigModel"],
  options: StandardComponentOptions,
  Component: StandardComponent(MultiViewApriltagRigModelElement),
  Element: MultiViewApriltagRigModelElement,
};
