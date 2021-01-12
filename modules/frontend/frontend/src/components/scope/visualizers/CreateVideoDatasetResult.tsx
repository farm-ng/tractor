/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CreateVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { CreateVideoDatasetConfigurationVisualizer } from "./CreateVideoDatasetConfiguration";
import { formatValue } from "../../../utils/formatValue";

const CreateVideoDatasetResultElement: React.FC<SingleElementVisualizerProps<
  CreateVideoDatasetResult
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const {
    configuration,
    perCameraNumFrames,
    perTagIdNumFrames,
    stampBegin,
    stampEnd,
    dataset,
  } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Stamp Begin", stampBegin],
            ["Stamp End", stampEnd],
            ["Dataset URL", dataset?.path],
          ]}
        />

        <KeyValueTable
          headers={["Camera Name", "Num Frames"]}
          records={perCameraNumFrames.map<[string, unknown]>((_) => [
            _.cameraName,
            _.numFrames,
          ])}
        />

        <KeyValueTable
          headers={["Tag ID", "Num Frames"]}
          records={perTagIdNumFrames.map<[string, unknown]>((_) => [
            formatValue(_.tagId),
            _.numFrames,
          ])}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          {
            <CreateVideoDatasetConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CreateVideoDatasetResultVisualizer = {
  id: "CreateVideoDatasetResult",
  types: ["type.googleapis.com/farm_ng.perception.CreateVideoDatasetResult"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CreateVideoDatasetResultElement),
  Element: CreateVideoDatasetResultElement,
};
