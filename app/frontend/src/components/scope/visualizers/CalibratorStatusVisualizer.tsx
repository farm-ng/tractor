/* eslint-disable no-console */
import * as React from "react";
import { Card, Table } from "react-bootstrap";
import {
  SingleElementVisualizerProps,
  Visualizer,
  VisualizerId,
  VisualizerOptionConfig,
  VisualizerProps
} from "../../../registry/visualization";
import {
  CalibratorStatus,
  MonocularApriltagRigModel,
  solverStatusToJSON
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrator";
import { formatValue } from "../../../utils/formatValue";
import { EventTypeId } from "../../../registry/events";
import { useEffect, useState } from "react";
import { Layout } from "./Layout";
import RangeSlider from "react-bootstrap-range-slider";
import { ApriltagDetectionsElement } from "./ApriltagDetectionsVisualizer";
import { ImageElement } from "./ImageVisualizer";
import { JsonPopover } from "../../JsonPopover";
import styles from "./CalibratorStatusVisualizer.module.scss";
import { Canvas } from "../../Canvas";
import { NamedSE3Pose } from "../../../../genproto/farm_ng_proto/tractor/v1/geometry";

import { Lights } from "../../Lights";
import { Controls } from "../../Controls";
import { ReferenceFrameViz } from "../../PoseViz";
import { Ground } from "../../Ground";
import { getDagTransform } from "../../../utils/geometry";

const CalibratorStatusElement: React.FC<SingleElementVisualizerProps<
  CalibratorStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  // Load the rig model asynchronously from the resource archive
  // TODO: useMemo instead of useState + useEffect?
  const [rigModel, setRigModel] = useState<MonocularApriltagRigModel | null>(
    null
  );
  useEffect(() => {
    const fetchRigModel = async (): Promise<void> => {
      const resource = value.apriltagRig?.rigModelResource;
      if (resources && resource) {
        try {
          const rigJson = await resources.getJson(resource.path);
          setRigModel(MonocularApriltagRigModel.fromJSON(rigJson));
        } catch (e) {
          console.error(`Error loading resource ${resource.path}: ${e}`);
        }
      }
    };
    fetchRigModel();
  }, [value, resources]);

  const {
    detections,
    cameraFrameName,
    cameraPosesRig,
    reprojectionImages,
    tagStats,
    rig
  } = rigModel || {};

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

  if (
    value.apriltagRig?.numFrames !== undefined &&
    !value.apriltagRig?.rigModelResource
  ) {
    return (
      <Card bg={"light"} className={"shadow-sm"}>
        <Card.Header>Summary</Card.Header>
        <Card.Body>
          <Table bordered size="sm" responsive="md">
            <tbody>
              <tr>
                <td> Frames captured </td>
                <td>{value.apriltagRig?.numFrames}</td>
              </tr>
            </tbody>
          </Table>
        </Card.Body>
        <Card.Footer className={styles.footer}>
          <span className="text-muted">{formatValue(new Date(timestamp))}</span>
          <JsonPopover json={value} />
        </Card.Footer>
      </Card>
    );
  }

  return (
    <Card bg={"light"} className={"shadow-sm"}>
      <Card.Body>
        {rigModel && (
          <Card>
            <Card.Header>Summary</Card.Header>
            <Card.Body>
              <Table bordered size="sm" responsive="md">
                <tbody>
                  <tr>
                    <td> Solver Status </td>
                    <td>{solverStatusToJSON(rigModel.solverStatus)}</td>
                  </tr>
                  <tr>
                    <td>Total RMSE</td>
                    <td>{formatValue(rigModel.rmse)}</td>
                  </tr>
                </tbody>
              </Table>
            </Card.Body>
          </Card>
        )}

        {tagStats && (
          <Card>
            <Card.Header>Tag Statistics</Card.Header>
            <Card.Body>
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
            </Card.Body>
          </Card>
        )}

        {markers && (
          <Card>
            <Card.Header>Apriltag Rig</Card.Header>
            <Card.Body>
              <Canvas>
                <Lights />
                <Ground />
                <fogExp2 args={[0xcccccc, 0.02]} />
                <Controls />
                {markers}
              </Canvas>
            </Card.Body>
          </Card>
        )}

        {(detection || reprojectionImage || tagRmses) && (
          <Card>
            <Card.Header>Details</Card.Header>
            <Card.Body>
              <RangeSlider
                value={index}
                onChange={(_, v) => setIndex(v)}
                min={0}
                max={
                  rigModel?.detections.length
                    ? rigModel?.detections.length - 1
                    : 0
                }
                step={1}
              />
              <div className={styles.detailRow}>
                {detection && (
                  // Unfortunately we don't have the timestamp for these detections
                  <ApriltagDetectionsElement
                    {...props}
                    value={[0, detection]}
                  />
                )}
                {reprojectionImage && (
                  <ImageElement {...props} value={[0, reprojectionImage]} />
                )}
              </div>
              <Card className={styles.card}>
                <Card.Body>
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
                </Card.Body>
              </Card>
            </Card.Body>
          </Card>
        )}
      </Card.Body>
      <Card.Footer className={styles.footer}>
        <span className="text-muted">{formatValue(new Date(timestamp))}</span>
        <JsonPopover json={value} />
      </Card.Footer>
    </Card>
  );
};

export class CalibratorStatusVisualizer
  implements Visualizer<CalibratorStatus> {
  static id: VisualizerId = "calibratorStatus";
  types: EventTypeId[] = [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibratorStatus"
  ];

  options: VisualizerOptionConfig[] = [
    { label: "view", options: ["overlay", "grid"] }
  ];

  component: React.FC<VisualizerProps<CalibratorStatus>> = (props) => {
    const view = props.options[0].value as "overlay" | "grid";
    return <Layout view={view} element={CalibratorStatusElement} {...props} />;
  };
}
