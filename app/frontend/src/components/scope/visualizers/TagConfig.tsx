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
import { TagConfig } from "../../../../genproto/farm_ng_proto/tractor/v1/apriltag";
import { KeyValueTable } from "./KeyValueTable";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";

const TagConfigForm: React.FC<FormProps<TagConfig>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <Form.Group
        label={`ID`}
        value={value.id}
        id={value.id.toString()}
        type="number"
        onChange={(e) => {
          const id = parseInt(e.target.value);
          setValue((v) => ({
            ...v,
            id
          }));
        }}
      />
      <Form.Group
        label="Size"
        value={value.size}
        id={`${value.id}_size`}
        type="number"
        onChange={(e) => {
          const size = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            size
          }));
        }}
      />
    </>
  );
};

const TagConfigElement: React.FC<SingleElementVisualizerProps<TagConfig>> = (
  props
) => {
  const {
    value: [timestamp, value]
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      <KeyValueTable records={[[value.id.toString(), value.size]]} />
    </Card>
  );
};

export const TagConfigVisualizer = {
  id: "TagConfig",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.TagConfig"],
  options: StandardComponentOptions,
  Component: StandardComponent(TagConfigElement),
  Element: TagConfigElement,
  Form: TagConfigForm
};
