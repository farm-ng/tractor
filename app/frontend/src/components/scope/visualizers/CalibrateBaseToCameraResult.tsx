/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CalibrateBaseToCameraResult } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_base_to_camera";
import {
  BaseToCameraModel,
  solverStatusToJSON
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrator";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { BaseToCameraModelVisualizer } from "./BaseToCameraModel";
import { CalibrateBaseToCameraConfigurationVisualizer } from "./CalibrateBaseToCameraConfiguration";

const CalibrateBaseToCameraResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateBaseToCameraResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const initial = useFetchResource<BaseToCameraModel>(
    value.baseToCameraModelInitial,
    resources
  );
  const solved = useFetchResource<BaseToCameraModel>(
    value.baseToCameraModelSolved,
    resources
  );

  const { configuration, solverStatus, rmse, stampEnd, eventLog } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatus && solverStatusToJSON(solverStatus)],
            ["Total RMSE", rmse],
            ["Stamp End", stampEnd],
            ["Event Log URL", eventLog?.path]
          ]}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          {
            <CalibrateBaseToCameraConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
      {initial && (
        <Card title="Initial">
          <BaseToCameraModelVisualizer.Element
            {...props}
            value={[0, initial]}
          />
        </Card>
      )}
      {solved && (
        <Card title="Solved">
          <BaseToCameraModelVisualizer.Element {...props} value={[0, solved]} />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateBaseToCameraResultVisualizer = {
  id: "CalibrateBaseToCameraResult",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraResult"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateBaseToCameraResultElement),
  Element: CalibrateBaseToCameraResultElement
};
