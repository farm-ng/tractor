/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { DetectApriltagsConfiguration } from "@farm-ng/genproto-perception/farm_ng/perception/detect_apriltags";
import {
  StandardComponent,
  StandardComponentOptions,
} from "./StandardComponent";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { CaptureVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { ApriltagConfig } from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import { ApriltagConfigVisualizer } from "./ApriltagConfig";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";

const DetectApriltagsConfigurationForm: React.FC<FormProps<
  DetectApriltagsConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

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
                "application/json; type=type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult",
            }),
          }));
        }}
      />
      <Form.Group
        // TODO: Replace with resource browser
        // TODO: Support rendering the apriltagconfig, behind toggle
        // TODO: Support creating a new apriltagconfig file, and editing the config here.
        label="ApriltagConfig Resource Path"
        value={value.tagConfig?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            tagConfig: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng.perception.ApriltagConfig",
            }),
          }));
        }}
      />
    </>
  );
};

const DetectApriltagsConfigurationElement: React.FC<SingleElementVisualizerProps<
  DetectApriltagsConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const { name } = value;
  const videoDataset = useFetchResource<CaptureVideoDatasetResult>(
    value.videoDataset,
    resources
  );
  const tagConfig = useFetchResource<ApriltagConfig>(
    value.tagConfig,
    resources
  );
  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable records={[["Name", name]]} />
      </Card>
      {videoDataset && (
        <Card title="Video Dataset">
          <CaptureVideoDatasetResultVisualizer.Element
            {...props}
            value={[0, videoDataset]}
          />
        </Card>
      )}
      {tagConfig && (
        <Card title="ApriltagConfig">
          <ApriltagConfigVisualizer.Element
            {...props}
            value={[0, tagConfig]}
          />
        </Card>
      )}
    </Card>
  );
};

export const DetectApriltagsConfigurationVisualizer = {
  id: "DetectApriltagsConfiguration",
  types: [
    "type.googleapis.com/farm_ng.perception.DetectApriltagsConfiguration",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(DetectApriltagsConfigurationElement),
  Element: DetectApriltagsConfigurationElement,
  Form: DetectApriltagsConfigurationForm,
};
