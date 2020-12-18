/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { Card } from "./Card";
import { CalibrateMultiViewApriltagRigResult } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_multi_view_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import {
  MultiViewApriltagRigModel,
  solverStatusToJSON,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { CalibrateMultiViewApriltagRigConfigurationVisualizer } from "./CalibrateMultiViewApriltagRigConfiguration";
import { KeyValueTable } from "./KeyValueTable";
import { MultiViewApriltagRigModelVisualizer } from "./MultiViewApriltagRigModel";
import { useStores } from "../../../hooks/useStores";

const CalibrateMultiViewApriltagRigResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const initial = useFetchResource<MultiViewApriltagRigModel>(
    value.multiViewApriltagRigInitial,
    resources
  );
  const solved = useFetchResource<MultiViewApriltagRigModel>(
    value.multiViewApriltagRigSolved,
    resources
  );

  const { baseUrl } = useStores();
  const blobstoreUrl = `${baseUrl}/blobstore`;

  const { configuration,  apriltagRigSolved, cameraRigSolved, solverStatus, rmse, stampBegin, stampEnd, eventLog } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Solver Status", solverStatus && solverStatusToJSON(solverStatus)],
            ["Total RMSE", rmse],
                        ["Stamp Begin", stampBegin],

            ["Stamp End", stampEnd],
            ["Event Log", eventLog?.path],
          ]}
        />
      </Card>
      {apriltagRigSolved && (
        <Card title="Apriltag Rig Output">
          <a target="_blank" href={`${blobstoreUrl}/${apriltagRigSolved.path}`}>
            {apriltagRigSolved.path}
          </a>
        </Card>
      )}
        {cameraRigSolved && (
        <Card title="Camera Rig Output">
          <a target="_blank" href={`${blobstoreUrl}/${cameraRigSolved.path}`}>
            {cameraRigSolved.path}
          </a>
        </Card>
      )}
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
  types: ["type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigResult"],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateMultiViewApriltagRigResultElement),
  Element: CalibrateMultiViewApriltagRigResultElement,
};
