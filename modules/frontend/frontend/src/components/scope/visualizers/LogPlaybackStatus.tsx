/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { Card } from "./Card";
import { LogPlaybackStatus } from "@farm-ng/genproto-core/farm_ng/core/log_playback";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { KeyValueTable } from "./KeyValueTable";
import { LogPlaybackConfigurationVisualizer } from "./LogPlaybackConfiguration";

const LogPlaybackStatusElement: React.FC<SingleElementVisualizerProps<
  LogPlaybackStatus
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const { configuration, lastMessageStamp, messageCount, messageStats } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Last Message Stamp", lastMessageStamp],
            ["Message Count", messageCount],
          ]}
        />
      </Card>
      {messageStats && (
        <Card title="Message Stats">
          <KeyValueTable
            headers={[
              "Name",
              "Type URL",
              "Count",
              "Last Stamp",
              "Avg. Frequency",
            ]}
            records={messageStats.map((entry) => [
              entry.name,
              entry.typeUrl,
              entry.count,
              entry.lastStamp,
              entry.frequency,
            ])}
          />
        </Card>
      )}

      {configuration && (
        <Card title="Configuration">
          {
            <LogPlaybackConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const LogPlaybackStatusVisualizer = {
  id: "LogPlaybackStatus",
  types: ["type.googleapis.com/farm_ng.core.LogPlaybackStatus"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(LogPlaybackStatusElement),
  Element: LogPlaybackStatusElement,
};
