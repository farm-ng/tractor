/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import {
  CalibrateIntrinsicsResult,
  CalibrateIntrinsicsStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_intrinsics";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { Card } from "./Card";
import { CalibrateIntrinsicsResultVisualizer } from "./CalibrateIntrinsicsResult";

const CalibrateIntrinsicsStatusElement: React.FC<SingleElementVisualizerProps<
  CalibrateIntrinsicsStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CalibrateIntrinsicsResult>(
    value.result,
    resources
  );

  if (!result) {
    return null;
  }

  return (
    <Card timestamp={timestamp} json={value}>
      <CalibrateIntrinsicsResultVisualizer.Element
        {...props}
        value={[0, result]}
      />
    </Card>
  );
};

export const CalibrateIntrinsicsStatusVisualizer = {
  id: "CalibrateIntrinsicsStatus",
  types: ["type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsStatus"],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateIntrinsicsStatusElement),
  Element: CalibrateIntrinsicsStatusElement,
};
