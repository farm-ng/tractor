/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";

import { Card } from "./Card";
import { CalibrateMultiViewApriltagRigStatus } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig";

const CalibrateMultiViewApriltagRigStatusElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigStatus
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

export const CalibrateMultiViewApriltagRigStatusVisualizer = {
  id: "CalibrateMultiViewApriltagRigStatus",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateMultiViewApriltagRigStatus"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateMultiViewApriltagRigStatusElement),
  Element: CalibrateMultiViewApriltagRigStatusElement
};
