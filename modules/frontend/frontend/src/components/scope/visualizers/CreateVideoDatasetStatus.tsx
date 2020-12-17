/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import {
  CreateVideoDatasetResult,
  CreateVideoDatasetStatus,
} from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  StandardComponent,
  StandardComponentOptions,
} from "./StandardComponent";
import { CreateVideoDatasetResultVisualizer } from "./CreateVideoDatasetResult";
import { formatValue } from "../../../utils/formatValue";

const CreateVideoDatasetStatusElement: React.FC<SingleElementVisualizerProps<
  CreateVideoDatasetStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CreateVideoDatasetResult>(
    value.result,
    resources
  );
  const { perCameraNumFrames, perTagIdNumFrames } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
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
      {result && (
        <Card title="Result">
          {
            <CreateVideoDatasetResultVisualizer.Element
              {...props}
              value={[0, result]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CreateVideoDatasetStatusVisualizer = {
  id: "CreateVideoDatasetStatus",
  types: ["type.googleapis.com/farm_ng.perception.CreateVideoDatasetStatus"],
  options: StandardComponentOptions,
  Component: StandardComponent(CreateVideoDatasetStatusElement),
  Element: CreateVideoDatasetStatusElement,
};
