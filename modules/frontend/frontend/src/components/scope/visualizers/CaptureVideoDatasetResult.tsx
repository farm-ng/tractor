/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import {
  StandardComponent,
  StandardComponentOptions,
} from "./StandardComponent";
import { CaptureVideoDatasetConfigurationVisualizer } from "./CaptureVideoDatasetConfiguration";
import { formatValue } from "../../../utils/formatValue";

const CaptureVideoDatasetResultElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetResult
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
            <CaptureVideoDatasetConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureVideoDatasetResultVisualizer = {
  id: "CaptureVideoDatasetResult",
  types: ["type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult"],
  options: StandardComponentOptions,
  Component: StandardComponent(CaptureVideoDatasetResultElement),
  Element: CaptureVideoDatasetResultElement,
};
