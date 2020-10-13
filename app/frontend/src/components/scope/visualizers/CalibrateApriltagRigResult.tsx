/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { CalibrateApriltagRigResult } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_apriltag_rig";
import {
  MonocularApriltagRigModel,
  solverStatusToJSON
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrator";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { MonocularApriltagRigModelVisualizer } from "./MonocularApriltagRigModel";
import { CalibrateApriltagRigConfigurationVisualizer } from "./CalibrateApriltagRigConfiguration";

const CalibrateApriltagRigResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateApriltagRigResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const initial = useFetchResource<MonocularApriltagRigModel>(
    value.monocularApriltagRigInitial,
    resources || undefined
  );
  const solved = useFetchResource<MonocularApriltagRigModel>(
    value.monocularApriltagRigSolved,
    resources || undefined
  );

  const { configuration, solverStatus, rmse, stampEnd } = value || {};
  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatus && solverStatusToJSON(solverStatus)],
            ["Total RMSE", rmse],
            ["Stamp End", stampEnd]
          ]}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          <CalibrateApriltagRigConfigurationVisualizer.Element
            {...props}
            value={[0, configuration]}
          />
        </Card>
      )}
      {initial && (
        <Card title="Initial">
          <MonocularApriltagRigModelVisualizer.Element
            {...props}
            value={[0, initial]}
          />
        </Card>
      )}
      {solved && (
        <Card title="Solved">
          <MonocularApriltagRigModelVisualizer.Element
            {...props}
            value={[0, solved]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateApriltagRigResultVisualizer = {
  id: "CalibrateApriltagRigResult",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigResult"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateApriltagRigResultElement),
  Element: CalibrateApriltagRigResultElement
};
