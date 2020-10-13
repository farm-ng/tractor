/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps
} from "../../../registry/visualization";
import {
  Quaternion,
  SE3Pose,
  Vec3
} from "../../../../genproto/farm_ng_proto/tractor/v1/geometry";
import { OverlayOptions, OverlayVisualizerComponent } from "./Overlay";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";

const SE3PoseForm: React.FC<FormProps<SE3Pose>> = (props) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <Form.Group
        label="tx"
        value={value.position?.x}
        type="number"
        onChange={(e) => {
          const x = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            position: { ...v.position, x } as Vec3
          }));
        }}
      />
      <Form.Group
        label="ty"
        value={value.position?.x}
        type="number"
        onChange={(e) => {
          const y = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            position: { ...v.position, y } as Vec3
          }));
        }}
      />
      <Form.Group
        label="tz"
        value={value.position?.z}
        type="number"
        onChange={(e) => {
          const z = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            position: { ...v.position, z } as Vec3
          }));
        }}
      />

      <Form.Group
        label="rx"
        value={value.rotation?.x}
        type="number"
        onChange={(e) => {
          const x = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            rotation: {
              ...v.rotation,
              x
            } as Quaternion
          }));
        }}
      />

      <Form.Group
        label="ry"
        value={value.rotation?.y}
        type="number"
        onChange={(e) => {
          const y = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            rotation: {
              ...v.rotation,
              y
            } as Quaternion
          }));
        }}
      />

      <Form.Group
        label="rz"
        value={value.rotation?.z}
        type="number"
        onChange={(e) => {
          const z = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            rotation: {
              ...v.rotation,
              z
            } as Quaternion
          }));
        }}
      />

      <Form.Group
        label="rw"
        value={value.rotation?.w}
        type="number"
        onChange={(e) => {
          const w = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            rotation: {
              ...v.rotation,
              w
            } as Quaternion
          }));
        }}
      />
    </>
  );
};

const SE3PoseElement: React.FC<SingleElementVisualizerProps<SE3Pose>> = () => {
  return <p>TODO</p>;
};

export const SE3PoseVisualizer = {
  id: "SE3Pose",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.SE3Pose"],
  options: OverlayOptions,
  Component: OverlayVisualizerComponent(SE3PoseElement),
  Element: SE3PoseElement,
  Form: SE3PoseForm
};
