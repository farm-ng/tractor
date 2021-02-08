import * as React from "react";
import { Card } from "./Card";

import { Vec2 } from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import {
  FormProps,
  SingleElementVisualizerProps,
  VisualizerProps,
} from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Vector3 } from "three";

const CustomVec2Element: React.FC<SingleElementVisualizerProps<Vec2>> = (
  props
) => {
  const {
    value: [timestamp, value],
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      <KeyValueTable
        records={[
          ["x", value.x],
          ["y", value.y],
        ]}
      />
    </Card>
  );
};

const CustomVec2MultiElement: React.FC<VisualizerProps<Vec2>> = (props) => {
  const { values } = props;

  const xs = values.map((_) => _[1].x);
  const ys = values.map((_) => _[1].y);
  const meanX = xs.reduce((a, b) => a + b) / xs.length;
  const meanY = ys.reduce((a, b) => a + b) / ys.length;

  return (
    <Card>
      <KeyValueTable
        records={[
          ["mean x", meanX],
          ["mean y", meanY],
        ]}
      />
    </Card>
  );
};

const CustomVec2Form: React.FC<FormProps<Vec2>> = (props) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <Form.Group
        label="x"
        value={value.x}
        type="number"
        onChange={(e) => {
          const x = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            x,
          }));
        }}
      />
      <Form.Group
        label="y"
        value={value.y}
        type="number"
        onChange={(e) => {
          const y = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            y,
          }));
        }}
      />
    </>
  );
};

const CustomVec2Element3D: React.FC<SingleElementVisualizerProps<Vec2>> = (
  props
) => {
  const {
    value: [, value],
  } = props;
  return (
    <group position={new Vector3(value.x, value.y, 0)}>
      <axesHelper />
    </group>
  );
};

export const CustomVec2Visualizer = {
  id: "CustomVec2",
  types: ["type.googleapis.com/farm_ng.perception.Vec2"],
  options: [],
  Element: CustomVec2Element,
  MultiElement: CustomVec2MultiElement,
  Element3D: CustomVec2Element3D,
  Form: CustomVec2Form,
};
