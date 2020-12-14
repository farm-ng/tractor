/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { Card } from "./Card";
import { KeyValueTable } from "./KeyValueTable";
import {
  CameraModel,
  cameraModel_DistortionModelToJSON as DistortionModelToJSON,
} from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";

const CameraModelElement: React.FC<SingleElementVisualizerProps<
  CameraModel
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      <KeyValueTable
        records={[
          ["Image Width", value.imageWidth],
          ["Image Height", value.imageHeight],
          ["cx", value.cx],
          ["cy", value.cy],
          ["fx", value.fx],
          ["fy", value.fy],
          ["Distortion Model", DistortionModelToJSON(value.distortionModel)],
          ["Distortion Coefficients", value.distortionCoefficients.join(", ")],
          ["Frame Name", value.frameName],
        ]}
      />
    </Card>
  );
};

export const CameraModelVisualizer = {
  id: "CameraModel",
  types: ["type.googleapis.com/farm_ng.perception.CameraModel"],
  options: StandardComponentOptions,
  Component: StandardComponent(CameraModelElement),
  Element: CameraModelElement,
};
