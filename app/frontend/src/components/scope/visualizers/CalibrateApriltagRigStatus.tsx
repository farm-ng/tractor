/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import {
  CalibrateApriltagRigResult,
  CalibrateApriltagRigStatus
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { Card } from "./Card";
import { CalibrateApriltagRigResultVisualizer } from "./CalibrateApriltagRigResult";

const CalibrateApriltagRigStatusElement: React.FC<SingleElementVisualizerProps<
  CalibrateApriltagRigStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
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
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigStatus"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateApriltagRigStatusElement),
  Element: CalibrateApriltagRigStatusElement
};
