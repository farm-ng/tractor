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
import { CalibrateApriltagRigConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureCalibrationDatasetResult } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "../../../../genproto/farm_ng_proto/tractor/v1/resource";
import { CaptureCalibrationDatasetResultVisualizer } from "./CaptureCalibrationDatasetResult";

CaptureCalibrationDatasetResultVisualizer.Element;

const CalibrateApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateApriltagRigConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  console.log(value);

  return (
    <>
      <Form.Group
        // TODO: Replace with resource browser
        label="Resource Path"
        value={value.calibrationDataset?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            calibrationDataset: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetResult"
            })
          }));
        }}
      />
      <h6>Tag IDs</h6>
      {value.tagIds.map((tagId, index) => (
        <React.Fragment key={index}>
          <Form.Group
            label={`Tag ID ${index}`}
            value={tagId}
            type="number"
            onChange={(e) => {
              const tagId = parseInt(e.target.value);
              setValue((v) => ({
                ...v,
                tagIds: Object.assign([...v.tagIds], { [index]: tagId })
              }));
            }}
          />
          <Form.ButtonGroup
            buttonText="X"
            onClick={() =>
              setValue((v) => ({
                ...v,
                tagIds: [
                  ...v.tagIds.slice(0, index),
                  ...v.tagIds.slice(index + 1)
                ]
              }))
            }
          />
        </React.Fragment>
      ))}

      <Form.ButtonGroup
        buttonText="+"
        onClick={() =>
          setValue((v) => ({
            ...v,
            tagIds: [...v.tagIds, 0]
          }))
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

      <Form.Group
        label="Root Tag ID"
        value={value.rootTagId}
        type="number"
        onChange={(e) => {
          const rootTagId = parseInt(e.target.value);
          setValue((v) => ({ ...v, rootTagId }));
        }}
      />

      <Form.Group
        label="Filter stable tags?"
        checked={value.filterStableTags}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const filterStableTags = Boolean(e.target.checked);
          setValue((v) => ({ ...v, filterStableTags }));
        }}
      />

      <Form.Group
        label="Camera Name"
        value={value.cameraName}
        type="text"
        onChange={(e) => {
          const cameraName = e.target.value;
          setValue((v) => ({ ...v, cameraName }));
        }}
      />
    </>
  );
};

const CalibrateApriltagRigConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateApriltagRigConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const { tagIds, rootTagId, name, filterStableTags, cameraName } = value;

  const calibrationDataset = useFetchResource<CaptureCalibrationDatasetResult>(
    value.calibrationDataset,
    resources
  );

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Name", name],
            ["Tag IDs", (tagIds || []).join(", ")],
            ["Root Tag ID", rootTagId],
            ["Filter Stable Tags?", filterStableTags],
            ["Camera Name", cameraName]
          ]}
        />
      </Card>
      {calibrationDataset && (
        <Card title="Calibration Dataset">
          <CaptureCalibrationDatasetResultVisualizer.Element
            {...props}
            value={[0, calibrationDataset]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateApriltagRigConfigurationVisualizer = {
  id: "CalibrateApriltagRigConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateApriltagRigConfigurationElement),
  Element: CalibrateApriltagRigConfigurationElement,
  Form: CalibrateApriltagRigConfigurationForm
};
