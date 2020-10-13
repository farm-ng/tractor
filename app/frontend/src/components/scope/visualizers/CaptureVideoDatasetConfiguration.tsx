/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps
} from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureVideoDatasetConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";
import {
  StandardComponent,
  StandardComponentOptions
} from "./StandardComponent";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";

const CaptureVideoDatasetConfigurationForm: React.FC<FormProps<
  CaptureVideoDatasetConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
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

const CaptureVideoDatasetConfigurationElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetConfiguration
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  const { name } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable records={[["Name", name]]} />
      </Card>
    </Card>
  );
};

export const CaptureVideoDatasetConfigurationVisualizer = {
  id: "CaptureVideoDatasetConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureVideoDatasetConfigurationElement),
  Element: CaptureVideoDatasetConfigurationElement,
  Form: CaptureVideoDatasetConfigurationForm
};
