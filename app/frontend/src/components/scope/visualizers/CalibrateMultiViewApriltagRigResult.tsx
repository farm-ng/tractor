/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { Card } from "./Card";
import { CalibrateMultiViewApriltagRigResult } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  MultiViewApriltagRigModel,
  solverStatusToJSON
} from "../../../../genproto/farm_ng_proto/tractor/v1/calibrator";
import { CalibrateMultiViewApriltagRigConfigurationVisualizer } from "./CalibrateMultiViewApriltagRigConfiguration";
import { KeyValueTable } from "./KeyValueTable";
import { MultiViewApriltagRigModelVisualizer } from "./MultiViewApriltagRigModel";

const CalibrateMultiViewApriltagRigResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const initial = useFetchResource<MultiViewApriltagRigModel>(
    value.multiViewApriltagRigInitial,
    resources
  );
  const solved = useFetchResource<MultiViewApriltagRigModel>(
    value.multiViewApriltagRigSolved,
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
            ["Event Log", eventLog?.path]
          ]}
        />
      </Card>
      {configuration && (
        <Card title="Configuration">
          <CalibrateMultiViewApriltagRigConfigurationVisualizer.Element
            {...props}
            value={[0, configuration]}
          />
        </Card>
      )}
      {initial && (
        <Card title="Initial">
          <MultiViewApriltagRigModelVisualizer.Element
            {...props}
            value={[0, initial]}
          />
        </Card>
      )}
      {solved && (
        <Card title="Solved">
          <MultiViewApriltagRigModelVisualizer.Element
            {...props}
            value={[0, solved]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigResultVisualizer = {
  id: "CalibrateMultiViewApriltagRigResult",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateMultiViewApriltagRigResult"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateMultiViewApriltagRigResultElement),
  Element: CalibrateMultiViewApriltagRigResultElement
};
