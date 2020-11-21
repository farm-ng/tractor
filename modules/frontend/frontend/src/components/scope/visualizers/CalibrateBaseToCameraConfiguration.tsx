/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CalibrateBaseToCameraConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_base_to_camera";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { CalibrateApriltagRigResult } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { BaseToCameraInitialization } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { CalibrateApriltagRigResultVisualizer } from "./CalibrateApriltagRigResult";
import { BaseToCameraInitializationVisualizer } from "./BaseToCameraInitialization";
import { CaptureVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";

const CalibrateBaseToCameraConfigurationForm: React.FC<FormProps<
  CalibrateBaseToCameraConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <Form.Group
        // TODO: Replace with resource browser
        label="Calibration Dataset"
        value={value.calibrationDataset?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            calibrationDataset: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult",
            }),
          }));
        }}
      />

      <Form.Group
        // TODO: Replace with resource browser
        label="Apriltag Rig Result"
        value={value.apriltagRigResult?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            apriltagRigResult: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigResult",
            }),
          }));
        }}
      />

      <BaseToCameraInitializationVisualizer.Form
        initialValue={
          value.initialization || BaseToCameraInitialization.fromPartial({})
        }
        onChange={(updated) =>
          setValue((v) => ({ ...v, initialization: updated }))
        }
      />

      <Form.Group
        label="Name"
        value={value.name}
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />
    </>
  );
};

const CalibrateBaseToCameraConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateBaseToCameraConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const calibrationDataset = useFetchResource<CaptureVideoDatasetResult>(
    value.calibrationDataset,
    resources
  );
  const apriltagRigResult = useFetchResource<CalibrateApriltagRigResult>(
    value.apriltagRigResult,
    resources
  );
  const { initialization, name } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable records={[["Name", name]]} />
      </Card>
      {initialization && (
        <Card title="base_pose_camera Initialization">
          <BaseToCameraInitializationVisualizer.Element
            value={[0, initialization]}
          />
        </Card>
      )}
      {calibrationDataset && (
        <Card title="Calibration Dataset">
          <CaptureVideoDatasetResultVisualizer.Element
            {...props}
            value={[0, calibrationDataset]}
          />
        </Card>
      )}
      {apriltagRigResult && (
        <Card title="Apriltag Rig Result">
          <CalibrateApriltagRigResultVisualizer.Element
            {...props}
            value={[0, apriltagRigResult]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateBaseToCameraConfigurationVisualizer = {
  id: "CalibrateBaseToCameraConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraConfiguration",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateBaseToCameraConfigurationElement),
  Element: CalibrateBaseToCameraConfigurationElement,
  Form: CalibrateBaseToCameraConfigurationForm,
};
