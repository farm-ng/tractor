/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import {
  CaptureRobotExtrinsicsDatasetResult,
  CaptureRobotExtrinsicsDatasetStatus,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/capture_robot_extrinsics_dataset";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  StandardMultiElement,
  StandardMultiElementOptions,
} from "./StandardMultiElement";
import { CaptureRobotExtrinsicsDatasetResultVisualizer } from "./CaptureRobotExtrinsicsDatasetResult";
import { CapturePoseRequestList } from "./CapturePoseRequestList";
import { formatValue } from "../../../utils/formatValue";
import { SE3Pose } from "@farm-ng/genproto-calibration/farm_ng/perception/geometry";
import { CapturePoseResponseVisualizer } from "./CapturePoseResponse";

function se3PoseToString(p: SE3Pose | undefined) {
  return [
    `tx: ${formatValue(p?.position?.x)}`,
    `ty: ${formatValue(p?.position?.y)}`,
    `tz: ${formatValue(p?.position?.z)}`,
    `rx: ${formatValue(p?.rotation?.x)}`,
    `ry: ${formatValue(p?.rotation?.y)}`,
    `rz: ${formatValue(p?.rotation?.z)}`,
    `rw: ${formatValue(p?.rotation?.w)}`,
  ].join(", ");
}

const CaptureRobotExtrinsicsDatasetStatusElement: React.FC<SingleElementVisualizerProps<
  CaptureRobotExtrinsicsDatasetStatus
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const result = useFetchResource<CaptureRobotExtrinsicsDatasetResult>(
    value.result,
    resources
  );
  const { configuration, latestRequestIndex, latestResponse } = value;
  const completedRequests =
    latestResponse === undefined ? 0 : latestRequestIndex + 1;

  return (
    <Card timestamp={timestamp} json={value}>
      {configuration && (
        <Card title="Progress">
          <KeyValueTable
            records={[
              ["Completed Requests", completedRequests],
              ["Total Requests", configuration?.requestQueue.length],
              ...(
                (latestResponse &&
                  configuration?.requestQueue[latestRequestIndex].poses) ||
                []
              ).map((_) => [
                `Latest ${_.frameA}_T_${_.frameB} request`,
                se3PoseToString(_.aPoseB),
              ]),
              ...(latestResponse?.poses || []).map((_) => [
                `Latest ${_.frameA}_T_${_.frameB} response`,
                se3PoseToString(_.aPoseB),
              ]),
              [
                "Latest joint state request",
                latestResponse &&
                  configuration?.requestQueue[latestRequestIndex].jointStates
                    .map((_) => `${_.name}: ${formatValue(_.value)}`)
                    .join(", "),
              ],
              [
                "Latest joint state response",
                latestResponse?.jointStates
                  .map((_) => `${_.name}: ${formatValue(_.value)}`)
                  .join(", "),
              ],
            ]}
          />
          {
            <CapturePoseRequestList
              requests={configuration.requestQueue}
              latestRequestIndex={
                latestResponse ? latestRequestIndex : undefined
              }
            />
          }
        </Card>
      )}
      {latestResponse && (
        <Card title="Latest Response">
          <CapturePoseResponseVisualizer.Element
            {...props}
            value={[timestamp, latestResponse]}
          />
        </Card>
      )}
      {result && (
        <Card title="Result">
          {
            <CaptureRobotExtrinsicsDatasetResultVisualizer.Element
              {...props}
              value={[timestamp, result]}
            />
          }
        </Card>
      )}
    </Card>
  );
};

export const CaptureRobotExtrinsicsDatasetStatusVisualizer = {
  id: "CaptureRobotExtrinsicsDatasetStatus",
  types: [
    "type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetStatus",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    CaptureRobotExtrinsicsDatasetStatusElement
  ),
  Element: CaptureRobotExtrinsicsDatasetStatusElement,
};
