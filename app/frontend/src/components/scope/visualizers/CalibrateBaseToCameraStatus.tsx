/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { Card } from "./Card";
import {
  CalibrateBaseToCameraResult,
  CalibrateBaseToCameraStatus
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_base_to_camera";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { CalibrateBaseToCameraResultVisualizer } from "./CalibrateBaseToCameraResult";

const CalibrateBaseToCameraStatusElement: React.FC<SingleElementVisualizerProps<
  CalibrateBaseToCameraStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const result = useFetchResource<CalibrateBaseToCameraResult>(
    value.result,
    resources
  );

  if (!result) {
    return null;
  }

  return (
    <Card timestamp={timestamp} json={value}>
      <CalibrateBaseToCameraResultVisualizer.Element
        {...props}
        value={[0, result]}
      />
    </Card>
  );
};

export const CalibrateBaseToCameraStatusVisualizer = {
  id: "CalibrateBaseToCameraStatus",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraStatus"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateBaseToCameraStatusElement),
  Element: CalibrateBaseToCameraStatusElement
};
