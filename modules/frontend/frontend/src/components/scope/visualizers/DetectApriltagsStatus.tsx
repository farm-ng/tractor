/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";
import { formatValue } from "../../../utils/formatValue";
import { DetectApriltagsStatus } from "@farm-ng/genproto-perception/farm_ng/perception/detect_apriltags";
import { DetectApriltagsConfigurationVisualizer } from "./DetectApriltagsConfiguration";

const DetectApriltagsStatusElement: React.FC<SingleElementVisualizerProps<
  DetectApriltagsStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CaptureVideoDatasetResult>(
    value.result,
    resources
  );
  const { perCameraNumFrames, perTagIdNumFrames, configuration } = value;

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
      {configuration && (
        <Card title="Configuration">
          {
            <DetectApriltagsConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const DetectApriltagsStatusVisualizer = {
  id: "DetectApriltagsStatus",
  types: ["type.googleapis.com/farm_ng.perception.DetectApriltagsStatus"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(DetectApriltagsStatusElement),
  Element: DetectApriltagsStatusElement,
};
