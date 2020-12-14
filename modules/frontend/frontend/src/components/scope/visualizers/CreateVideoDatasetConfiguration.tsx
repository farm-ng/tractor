/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import {
  CreateVideoDatasetConfiguration,
  VideoFileCamera,
} from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import {
  StandardComponent,
  StandardComponentOptions,
} from "./StandardComponent";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { VideoFileCameraVisualizer } from "./VideoFileCamera";

const CreateVideoDatasetConfigurationForm: React.FC<FormProps<
  CreateVideoDatasetConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  const { videoFileCameras, apriltagRigs } = value;

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
        label="Detect Apriltags?"
        checked={value.detectApriltags}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const detectApriltags = Boolean(e.target.checked);
          setValue((v) => ({ ...v, detectApriltags }));
        }}
      />

      {videoFileCameras?.map((videoFileCamera, index) => (
        <div key={videoFileCamera.cameraFrameName}>
          <VideoFileCameraVisualizer.Form
            initialValue={videoFileCamera}
            onChange={(updated) =>
              setValue((v) => ({
                ...v,
                videoFileCameras: Object.assign([...v.videoFileCameras], {
                  [index]: updated,
                }),
              }))
            }
          />
          <Form.ButtonGroup
            buttonText="X"
            onClick={() =>
              setValue((v) => ({
                ...v,
                videoFileCameras: [
                  ...(v.videoFileCameras || []).slice(0, index),
                  ...(v.videoFileCameras || []).slice(index + 1),
                ],
              }))
            }
          />
        </div>
      ))}

      <Form.ButtonGroup
        buttonText="+"
        onClick={() =>
          setValue((v) => ({
            ...v,
            videoFileCameras: [
              ...(v.videoFileCameras || []),
              VideoFileCamera.fromPartial({}),
            ],
          }))
        }
      />

      {apriltagRigs?.map((apriltagRig, index) => (
        <div key={apriltagRig.path}>
          <Form.Group
            // TODO: Replace with resource browser
            label="Resource Path"
            value={apriltagRig.path}
            type="text"
            onChange={(e) => {
              const path = e.target.value;
              setValue((v) => ({
                ...v,
                apriltagRigs: Object.assign([...v.apriltagRigs], {
                  [index]: Resource.fromPartial({
                    path,
                    contentType:
                      "application/json; type=type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigResult",
                  }),
                }),
              }));
            }}
          />
          <Form.ButtonGroup
            buttonText="X"
            onClick={() =>
              setValue((v) => ({
                ...v,
                apriltagRigs: [
                  ...(v.apriltagRigs || []).slice(0, index),
                  ...(v.apriltagRigs || []).slice(index + 1),
                ],
              }))
            }
          />
        </div>
      ))}

      <Form.ButtonGroup
        buttonText="+"
        onClick={() =>
          setValue((v) => ({
            ...v,
            apriltagRigs: [...(v.apriltagRigs || []), Resource.fromPartial({})],
          }))
        }
      />
    </>
  );
};

const CreateVideoDatasetConfigurationElement: React.FC<SingleElementVisualizerProps<
  CreateVideoDatasetConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const { name } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable records={[["Name", name]]} />
      </Card>
    </Card>
  );
};

export const CreateVideoDatasetConfigurationVisualizer = {
  id: "CreateVideoDatasetConfiguration",
  types: [
    "type.googleapis.com/farm_ng.perception.CreateVideoDatasetConfiguration",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CreateVideoDatasetConfigurationElement),
  Element: CreateVideoDatasetConfigurationElement,
  Form: CreateVideoDatasetConfigurationForm,
};
