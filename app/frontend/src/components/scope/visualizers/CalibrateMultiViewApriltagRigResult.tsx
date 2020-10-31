/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { Card } from "./Card";
import { CalibrateMultiViewApriltagRigResult } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig";

const CalibrateMultiViewApriltagRigResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigResult
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;
  return (
    <Card timestamp={timestamp} json={value}>
      TODO
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigResultVisualizer = {
  id: "CalibrateMultiViewApriltagRigResult",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateMultiViewApriltagRigResult"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateMultiViewApriltagRigResultElement),
  Element: CalibrateMultiViewApriltagRigResultElement
};
