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
import styles from "./TagConfig.module.scss";

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
      {/*
      Some black magic...
      In a controlled form like this one, entering decimal points is tricky because
      as the user types "0.", this will be parsed to the value 0, and rendered as such (obliterating the decimal point).
      Somehow, this combination of a "number" form field, a String value, and a Number state seems to work.
      https://stackoverflow.com/a/47663989
      */}
      <Form.Group
        label="Size"
        value={String(value.size)}
        id={`${value.id}_size`}
        type="number"
        className={styles.tagSizeInput}
        onChange={(e) => {
          const size = Number(e.target.value);
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
