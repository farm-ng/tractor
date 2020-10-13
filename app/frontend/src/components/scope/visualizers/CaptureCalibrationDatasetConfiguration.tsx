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
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureCalibrationDatasetConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";

const CaptureCalibrationDatasetConfigurationForm: React.FC<FormProps<
  CaptureCalibrationDatasetConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <Form.Group
        label="Number of Frames"
        value={value.numFrames}
        type="number"
        onChange={(e) => {
          const numFrames = parseInt(e.target.value);
          setValue((v) => ({ ...v, numFrames }));
        }}
      />

      <Form.Group
        label="Name"
        value={value.name}
        description="A name for the dataset, used to name the output archive."
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />
    </>
  );
};

const CaptureCalibrationDatasetConfigurationElement: React.FC<SingleElementVisualizerProps<
  CaptureCalibrationDatasetConfiguration
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  const { numFrames, name } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Num Frames", numFrames],
            ["Name", name]
          ]}
        />
      </Card>
    </Card>
  );
};

export const CaptureCalibrationDatasetConfigurationVisualizer = {
  id: "CaptureCalibrationDatasetConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureCalibrationDatasetConfigurationElement),
  Element: CaptureCalibrationDatasetConfigurationElement,
  Form: CaptureCalibrationDatasetConfigurationForm
};
