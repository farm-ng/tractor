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
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { toQuaternion, toVector3 } from "../../../utils/protoConversions";
import {
  Standard3DComponent,
  Standard3DComponentOptions,
  Standard3DElement
} from "./StandardComponent";

const SE3PoseMarker3D: React.FC<SingleElementVisualizerProps<SE3Pose>> = (
  props
) => {
  const {
    value: [, value]
  } = props;
  return (
    <group
      position={toVector3(value.position)}
      quaternion={toQuaternion(value.rotation)}
    >
      <axesHelper />
    </group>
  );
};

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

export const SE3PoseVisualizer = {
  id: "SE3Pose",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.SE3Pose"],
  options: Standard3DComponentOptions,
  Component: Standard3DComponent(SE3PoseMarker3D),
  Element: Standard3DElement(SE3PoseMarker3D),
  Form: SE3PoseForm,
  Marker3D: SE3PoseMarker3D
};
