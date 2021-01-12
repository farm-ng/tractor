/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CreateVideoDatasetConfiguration } from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { VideoFileCameraVisualizer } from "./VideoFileCamera";
import { ResourceVisualizer } from "./Resource";
import { useStableKey } from "../../../hooks/useStableKey";

const CreateVideoDatasetConfigurationForm: React.FC<FormProps<
  CreateVideoDatasetConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  const { apriltagRigs } = value;
  const keyedVideoFileCameras = useStableKey(value.videoFileCameras);

  return (
    <>
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
        label="Detect Apriltags?"
        checked={value.detectApriltags}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const detectApriltags = Boolean(e.target.checked);
          setValue((v) => ({ ...v, detectApriltags }));
        }}
      />

      {keyedVideoFileCameras?.map(([key, videoFileCamera], index) => (
        <div key={key}>
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
          {/* Disabled until multiple video file inputs are supported

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
          /> */}
        </div>
      ))}

      {/* Disabled until multiple video file inputs are supported
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
      /> */}

      {apriltagRigs?.map((apriltagRig, index) => (
        <div key={apriltagRig.path}>
          <ResourceVisualizer.Form
            label="Apriltag Rig"
            initialValue={Resource.fromPartial({
              path: apriltagRig.path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng.perception.ApriltagRig",
            })}
            onChange={(updated) =>
              setValue((v) => ({
                ...v,
                apriltagRigs: Object.assign([...v.apriltagRigs], {
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
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CreateVideoDatasetConfigurationElement),
  Element: CreateVideoDatasetConfigurationElement,
  Form: CreateVideoDatasetConfigurationForm,
};
