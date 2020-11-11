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
import { CalibrateMultiViewApriltagRigConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "../../../../genproto/farm_ng_proto/tractor/v1/resource";
import { RepeatedIntForm } from "./RepeatedIntForm";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";
import { CaptureVideoDatasetResult } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_video_dataset";

const CalibrateMultiViewApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <Form.Group
        // TODO: Replace with resource browser
        label="Video Dataset Resource Path"
        value={value.videoDataset?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            videoDataset: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetResult"
            })
          }));
        }}
      />

      <h6>Tag IDs</h6>
      <RepeatedIntForm
        initialValue={value.tagIds}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            tagIds: updated
          }))
        }
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
        label="Root Camera Name"
        value={value.rootCameraName}
        type="text"
        onChange={(e) => {
          const rootCameraName = e.target.value;
          setValue((v) => ({ ...v, rootCameraName }));
        }}
      />

      <Form.Group
        label="Name"
        value={value.name}
        description="A name for the calibration, used to name the output archive."
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />

      <Form.Group
        label="Tag Rig Name"
        value={value.tagRigName}
        type="text"
        onChange={(e) => {
          const tagRigName = e.target.value;
          setValue((v) => ({ ...v, tagRigName }));
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
    </>
  );
};

const CalibrateMultiViewApriltagRigConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const { tagIds, rootTagId, rootCameraName, name, tagRigName } = value;

  const videoDataset = useFetchResource<CaptureVideoDatasetResult>(
    value.videoDataset,
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
            ["Root Camera Name", rootCameraName],
            ["Tag Rig Name", tagRigName]
          ]}
        />
      </Card>
      {videoDataset && (
        <Card title="Video Dataset">
          <CaptureVideoDatasetResultVisualizer.Element
            {...props}
            value={[0, videoDataset]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigConfigurationVisualizer = {
  id: "CalibrateMultiViewApriltagRigConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateMultiViewApriltagRigConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(
    CalibrateMultiViewApriltagRigConfigurationElement
  ),
  Element: CalibrateMultiViewApriltagRigConfigurationElement,
  Form: CalibrateMultiViewApriltagRigConfigurationForm
};
