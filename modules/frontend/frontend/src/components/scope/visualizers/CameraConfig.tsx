/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { Card } from "./Card";
import { KeyValueTable } from "./KeyValueTable";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { enumNumericKeys } from "../../../utils/enum";
import {
  CameraConfig,
  CameraConfig_Model as Model,
  cameraConfig_ModelToJSON as ModelToJSON,
} from "@farm-ng/genproto-perception/farm_ng/perception/camera_pipeline";

const CameraConfigForm: React.FC<FormProps<CameraConfig>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <Form.Group
        label="Name"
        value={value.name}
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />

      <Form.Group
        label="Serial Number"
        value={value.serialNumber}
        type="text"
        onChange={(e) => {
          const serialNumber = e.target.value;
          setValue((v) => ({ ...v, serialNumber }));
        }}
      />

      <Form.Group
        label="Model"
        value={value.model}
        as="select"
        onChange={(e) => {
          const model = parseInt(e.target.value);
          setValue((v) => ({ ...v, model }));
        }}
      >
        {enumNumericKeys(Model)
          .filter((k) => k >= 0)
          .map((k) => {
            return (
              <option key={k} value={k}>
                {ModelToJSON(k)}
              </option>
            );
          })}
      </Form.Group>

      <Form.Group
        label="UDP Stream Port"
        value={value.udpStreamPort}
        type="number"
        onChange={(e) => {
          const udpStreamPort = parseInt(e.target.value);
          setValue((v) => ({
            ...v,
            udpStreamPort,
          }));
        }}
      />
    </>
  );
};

const CameraConfigElement: React.FC<SingleElementVisualizerProps<
  CameraConfig
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      <KeyValueTable
        records={[
          ["Name", value.name],
          ["Serial Number", value.serialNumber],
          ["Model", ModelToJSON(value.model)],
          ["UDP Stream Port", value.udpStreamPort],
        ]}
      />
    </Card>
  );
};

export const CameraConfigVisualizer = {
  id: "CameraConfig",
  types: ["type.googleapis.com/farm_ng.perception.CameraConfig"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CameraConfigElement),
  Element: CameraConfigElement,
  Form: CameraConfigForm,
};
