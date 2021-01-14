/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import {
  CalibrateApriltagRigResult,
  CalibrateApriltagRigStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { Card } from "./Card";
import { CalibrateApriltagRigResultVisualizer } from "./CalibrateApriltagRigResult";

const CalibrateApriltagRigStatusElement: React.FC<SingleElementVisualizerProps<
  CalibrateApriltagRigStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CalibrateApriltagRigResult>(
    value.result,
    resources
  );

  if (!result) {
    return null;
  }

  return (
    <Card timestamp={timestamp} json={value}>
      <CalibrateApriltagRigResultVisualizer.Element
        {...props}
        value={[0, result]}
      />
    </Card>
  );
};

export const CalibrateApriltagRigStatusVisualizer = {
  id: "CalibrateApriltagRigStatus",
  types: ["type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigStatus"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CalibrateApriltagRigStatusElement),
  Element: CalibrateApriltagRigStatusElement,
};
