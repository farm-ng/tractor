/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps
} from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { Card } from "./Card";
import { CalibrateMultiViewApriltagRigConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig";

const CalibrateMultiViewApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = () => {
  return <>TODO</>;
};

const CalibrateMultiViewApriltagRigConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigConfiguration
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

export const CalibrateMultiViewApriltagRigConfigurationVisualizer = {
  id: "CalibrateMultiViewApriltagRigConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateMultiViewApriltagRigConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(
    CalibrateMultiViewApriltagRigConfigurationElement
  ),
  Element: CalibrateMultiViewApriltagRigConfigurationElement,
  Form: CalibrateMultiViewApriltagRigConfigurationForm
};
