/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import {
  NamedSE3Pose,
  SE3Pose,
} from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import { toQuaternion, toVector3 } from "../../../utils/protoConversions";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { SE3PoseVisualizer } from "./SE3Pose";
import { Html } from "drei";
import {
  Standard3DComponent,
  Standard3DComponentOptions,
  Standard3DElement,
} from "./StandardComponent";

const NamedSE3Pose3DElement: React.FC<SingleElementVisualizerProps<
  NamedSE3Pose
>> = ({ children, ...props }) => {
  const {
    value: [, value],
  } = props;

  return (
    <group>
      <line>
        <geometry
          attach="geometry"
          vertices={[toVector3(undefined), toVector3(value.aPoseB?.position)]}
          onUpdate={(self) => (self.verticesNeedUpdate = true)}
        />
        <lineBasicMaterial attach="material" color="lightgray" />
      </line>
      <group
        position={toVector3(value.aPoseB?.position)}
        quaternion={toQuaternion(value.aPoseB?.rotation)}
      >
        <axesHelper scale={[0.2, 0.2, 0.2]}>
          <Html>
            <div>{value.frameB}</div>
          </Html>
        </axesHelper>
        {children}
      </group>
    </group>
  );
};

const NamedSE3PoseForm: React.FC<FormProps<NamedSE3Pose>> = (props) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <Form.Group
        label="Frame A"
        value={value.frameA}
        type="text"
        onChange={(e) => {
          const frameA = e.target.value;
          setValue((v) => ({ ...v, frameA }));
        }}
      />
      <Form.Group
        label="Frame B"
        value={value.frameB}
        type="text"
        onChange={(e) => {
          const frameB = e.target.value;
          setValue((v) => ({ ...v, frameB }));
        }}
      />

      <SE3PoseVisualizer.Form
        initialValue={value.aPoseB || SE3Pose.fromJSON({})}
        onChange={(updated) => setValue((v) => ({ ...v, aPoseB: updated }))}
      />
    </>
  );
};

export const NamedSE3PoseVisualizer = {
  id: "NamedSE3Pose",
  types: ["type.googleapis.com/farm_ng.perception.NamedSE3Pose"],
  options: Standard3DComponentOptions,
  Component: Standard3DComponent(NamedSE3Pose3DElement),
  Element: Standard3DElement(NamedSE3Pose3DElement),
  Form: NamedSE3PoseForm,
  Marker3D: NamedSE3Pose3DElement,
};
