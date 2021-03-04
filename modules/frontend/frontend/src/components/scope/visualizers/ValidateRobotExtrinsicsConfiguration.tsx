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
import { ValidateRobotExtrinsicsConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/validate_robot_extrinsics";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { ResourceVisualizer } from "./Resource";
import { ApriltagRigVisualizer } from "./ApriltagRig";
import { ApriltagRig } from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
// import { useStores } from "../../../hooks/useStores";

const ValidateRobotExtrinsicsConfigurationForm: React.FC<FormProps<
  ValidateRobotExtrinsicsConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);
  // const { httpResourceArchive } = useStores();

  // const loadedDataset = useFetchResource<CaptureRobotExtrinsicsDatasetResult>(
  //   value.dataset,
  //   httpResourceArchive
  // );

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

      <ResourceVisualizer.Form
        initialValue={Resource.fromPartial({
          path: value.apriltagRig?.path,
          contentType:
            "application/json; type=type.googleapis.com/farm_ng.perception.ApriltagRig",
        })}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            apriltagRig: updated,
          }))
        }
      />
      <Form.Group
        label="HAL Service Address"
        value={value.halServiceAddress}
        description="host:port of the gRPC RobotHal service."
        type="text"
        onChange={(e) => {
          const halServiceAddress = e.target.value;
          setValue((v) => ({ ...v, halServiceAddress }));
        }}
      />
    </>
  );
};

const ValidateRobotExtrinsicsConfigurationElement: React.FC<SingleElementVisualizerProps<
  ValidateRobotExtrinsicsConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const { name, halServiceAddress } = value;

  const apriltagRig = useFetchResource<ApriltagRig>(
    value.apriltagRig,
    resources
  );

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Name", name],
            ["HAL Service Address", halServiceAddress],
          ]}
        />
      </Card>
      {apriltagRig && (
        <Card title="Apriltag Rig">
          <ApriltagRigVisualizer.Element {...props} value={[0, apriltagRig]} />
        </Card>
      )}
    </Card>
  );
};

export const ValidateRobotExtrinsicsConfigurationVisualizer = {
  id: "ValidateRobotExtrinsicsConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.ValidateRobotExtrinsicsConfiguration",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    ValidateRobotExtrinsicsConfigurationElement
  ),
  Element: ValidateRobotExtrinsicsConfigurationElement,
  Form: ValidateRobotExtrinsicsConfigurationForm,
};
