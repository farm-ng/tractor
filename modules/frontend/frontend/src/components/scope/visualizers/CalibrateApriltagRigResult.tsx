/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { CalibrateApriltagRigResult } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_apriltag_rig";
import {
  MonocularApriltagRigModel,
  solverStatusToJSON,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
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
    resources,
  } = props;

  const initial = useFetchResource<MonocularApriltagRigModel>(
    value.monocularApriltagRigInitial,
    resources
  );
  const solved = useFetchResource<MonocularApriltagRigModel>(
    value.monocularApriltagRigSolved,
    resources
  );

  const { configuration, solverStatus, rmse, stampEnd } = value;
  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatus && solverStatusToJSON(solverStatus)],
            ["Total RMSE", rmse],
            ["Stamp End", stampEnd],
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
  types: ["type.googleapis.com/farm_ng.calibration.CalibrateApriltagRigResult"],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateApriltagRigResultElement),
  Element: CalibrateApriltagRigResultElement,
};
