/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import {
  CaptureVideoDatasetResult,
  CaptureVideoDatasetStatus,
} from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";
import { formatValue } from "../../../utils/formatValue";

const CaptureVideoDatasetStatusElement: React.FC<SingleElementVisualizerProps<
  CaptureVideoDatasetStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CaptureVideoDatasetResult>(
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
            <CaptureVideoDatasetResultVisualizer.Element
              {...props}
              value={[0, result]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureVideoDatasetStatusVisualizer = {
  id: "CaptureVideoDatasetStatus",
  types: ["type.googleapis.com/farm_ng.perception.CaptureVideoDatasetStatus"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CaptureVideoDatasetStatusElement),
  Element: CaptureVideoDatasetStatusElement,
};
