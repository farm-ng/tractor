import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { VideoFileCamera } from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import * as React from "react";
import { useFormState } from "../../../hooks/useFormState";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import Form from "./Form";
import { KeyValueTable } from "./KeyValueTable";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";

const VideoFileCameraForm: React.FC<FormProps<VideoFileCamera>> = (props) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <Form.Group
        label="Camera Frame Name"
        value={value.cameraFrameName}
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />

      <Form.Group
        // TODO: Replace with resource browser
        label="Video File Resource"
        value={value.videoFileResource?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            videoFileResource: Resource.fromPartial({
              path,
              contentType: "video/mp4",
            }),
          }));
        }}
      />
    </>
  );
};

const VideoFileCameraElement: React.FC<SingleElementVisualizerProps<
  VideoFileCamera
>> = ({ value: [, value] }) => {
  const { cameraFrameName, videoFileResource } = value;
  return (
    <>
      <KeyValueTable
        records={[
          ["Camera Frame Name", cameraFrameName],
          ["Video File Resource", videoFileResource?.path],
        ]}
      />
    </>
  );
};

export const VideoFileCameraVisualizer = {
  id: "VideoFileCamera",
  types: ["type.googleapis.com/farm_ng.perception.VideoFileCamera"],
  options: StandardComponentOptions,
  Component: StandardComponent(VideoFileCameraElement),
  Element: VideoFileCameraElement,
  Form: VideoFileCameraForm,
};
