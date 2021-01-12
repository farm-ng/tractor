/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";

import { Card } from "./Card";
import {
  CalibrateMultiViewApriltagRigResult,
  CalibrateMultiViewApriltagRigStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_multi_view_apriltag_rig";
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
  types: [
    "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigStatus",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    CalibrateMultiViewApriltagRigStatusElement
  ),
  Element: CalibrateMultiViewApriltagRigStatusElement,
};
