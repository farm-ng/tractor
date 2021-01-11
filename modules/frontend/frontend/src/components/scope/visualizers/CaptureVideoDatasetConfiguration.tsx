/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureVideoDatasetConfiguration } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
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
      <Form.Group
        label="Detect Apriltags?"
        checked={value.detectApriltags}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const detectApriltags = Boolean(e.target.checked);
          setValue((v) => ({ ...v, detectApriltags }));
        }}
      />
    </>
  );
};

const CaptureVideoDatasetConfigurationElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
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
    "type.googleapis.com/farm_ng.perception.CaptureVideoDatasetConfiguration",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CaptureVideoDatasetConfigurationElement),
  Element: CaptureVideoDatasetConfigurationElement,
  Form: CaptureVideoDatasetConfigurationForm,
};
