/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";

import { Card } from "./Card";
import {
  CalibrateMultiViewApriltagRigResult,
  CalibrateMultiViewApriltagRigStatus,
} from "@farm-ng/genproto/farm_ng/v1/calibrate_multi_view_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { CalibrateMultiViewApriltagRigResultVisualizer } from "./CalibrateMultiViewApriltagRigResult";

const CalibrateMultiViewApriltagRigStatusElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CalibrateMultiViewApriltagRigResult>(
    value.result,
    resources
  );

  if (!result) {
    return null;
  }

  return (
    <Card timestamp={timestamp} json={value}>
      <CalibrateMultiViewApriltagRigResultVisualizer.Element
        {...props}
        value={[0, result]}
      />
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigStatusVisualizer = {
  id: "CalibrateMultiViewApriltagRigStatus",
  types: ["type.googleapis.com/farm_ng.v1.CalibrateMultiViewApriltagRigStatus"],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateMultiViewApriltagRigStatusElement),
  Element: CalibrateMultiViewApriltagRigStatusElement,
};
