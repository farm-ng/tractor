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
import { CalibrateApriltagRigConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { RepeatedIntForm } from "./RepeatedIntForm";
import { CaptureVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";

const CalibrateApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateApriltagRigConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

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
                "application/json; type=type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult",
            }),
          }));
        }}
      />

      <h6>Tag IDs</h6>
      <RepeatedIntForm
        initialValue={value.tagIds}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            tagIds: updated,
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
    resources,
  } = props;

  const { tagIds, rootTagId, name, filterStableTags, cameraName } = value;

  const calibrationDataset = useFetchResource<CaptureVideoDatasetResult>(
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
            ["Camera Name", cameraName],
          ]}
        />
      </Card>
      {calibrationDataset && (
        <Card title="Calibration Dataset">
          <CaptureVideoDatasetResultVisualizer.Element
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
    "type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigConfiguration",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateApriltagRigConfigurationElement),
  Element: CalibrateApriltagRigConfigurationElement,
  Form: CalibrateApriltagRigConfigurationForm,
};
