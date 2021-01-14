/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import { Card } from "./Card";
import { LogPlaybackConfiguration } from "@farm-ng/genproto-core/farm_ng/core/log_playback";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { ResourceVisualizer } from "./Resource";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { KeyValueTable } from "./KeyValueTable";

const LogPlaybackConfigurationForm: React.FC<FormProps<
  LogPlaybackConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <ResourceVisualizer.Form
        label="Log"
        initialValue={Resource.fromPartial({
          path: value.log?.path,
          contentType: "application/farm_ng.eventlog.v1",
        })}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            log: updated,
          }))
        }
      />
      <Form.Group
        label="Loop"
        description="Loop playback?"
        checked={value.loop}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const loop = Boolean(e.target.checked);
          setValue((v) => ({ ...v, loop }));
        }}
      />
      <Form.Group
        label="Send"
        description="Publish on the bus?"
        checked={value.send}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const send = Boolean(e.target.checked);
          setValue((v) => ({ ...v, send }));
        }}
      />
      <Form.Group
        label="speed"
        description="Playback speed (multiple of realtime)"
        value={value.speed}
        type="number"
        onChange={(e) => {
          const speed = parseFloat(e.target.value);
          setValue((v) => ({ ...v, speed }));
        }}
      />
    </>
  );
};

const LogPlaybackConfigurationElement: React.FC<SingleElementVisualizerProps<
  LogPlaybackConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Log Path", value.log?.path],
            ["Loop", value.loop],
            ["Send", value.send],
            ["Speed", value.speed],
          ]}
        />
      </Card>
    </Card>
  );
};

export const LogPlaybackConfigurationVisualizer = {
  id: "LogPlaybackConfiguration",
  types: ["type.googleapis.com/farm_ng.core.LogPlaybackConfiguration"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(LogPlaybackConfigurationElement),
  Element: LogPlaybackConfigurationElement,
  Form: LogPlaybackConfigurationForm,
};
