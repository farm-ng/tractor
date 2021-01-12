/* eslint-disable no-console */
import * as React from "react";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { IntrinsicModel } from "@farm-ng/genproto-calibration/farm_ng/calibration/intrinsic_model";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { KeyValueTable } from "./KeyValueTable";
import { Table } from "react-bootstrap";
import { solverStatusToJSON } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { formatValue } from "../../../utils/formatValue";
import { CameraModelVisualizer } from "./CameraModel";
import { useState } from "react";
import RangeSlider from "react-bootstrap-range-slider";
import { ImageVisualizer } from "./Image";
import { ApriltagDetectionsVisualizer } from "./ApriltagDetections";

const IntrinsicModelElement: React.FC<SingleElementVisualizerProps<
  IntrinsicModel
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const {
    tagStats,
    cameraModel,
    cameraPosesRig,
    detections,
    solverStatus,
    rmse,
  } = value;

  // The currently selected image index
  const [index, setIndex] = useState(0);
  const maxImageIndex = cameraPosesRig.length ? cameraPosesRig.length - 1 : 0;

  const reprojectionImage = value.reprojectionImages[index];
  const apriltagDetections = detections[index];

  return (
    <Card json={value} timestamp={timestamp}>
      <Card title="Camera Model">
        {cameraModel && (
          <CameraModelVisualizer.Element {...props} value={[0, cameraModel]} />
        )}
      </Card>

      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatusToJSON(solverStatus)],
            ["Total RMSE", rmse],
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

        <Card title="Reprojection Images">
          {reprojectionImage && (
            <ImageVisualizer.Element
              {...props}
              value={[0, reprojectionImage]}
            />
          )}
        </Card>

        <Card title="Apriltag Detections">
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

export const IntrinsicModelVisualizer = {
  id: "IntrinsicModel",
  types: ["type.googleapis.com/farm_ng.calibration.IntrinsicModel"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(IntrinsicModelElement),
  Element: IntrinsicModelElement,
};
