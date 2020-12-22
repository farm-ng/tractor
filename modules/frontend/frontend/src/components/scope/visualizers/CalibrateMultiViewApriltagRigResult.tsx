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
import { Button, Table } from "react-bootstrap";
import { Icon } from "../../Icon";
import { JsonPopover } from "../../JsonPopover";
import { ApriltagRig } from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import { MultiViewCameraRig } from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";

const CalibrateMultiViewApriltagRigResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const loadedInitial = useFetchResource<MultiViewApriltagRigModel>(
    value.multiViewApriltagRigInitial,
    resources
  );
  const loadedSolved = useFetchResource<MultiViewApriltagRigModel>(
    value.multiViewApriltagRigSolved,
    resources
  );
  const loadedCameraRig = useFetchResource<MultiViewCameraRig>(
    value.cameraRigSolved,
    resources
  );
  const loadedApriltagRig = useFetchResource<ApriltagRig>(
    value.apriltagRigSolved,
    resources
  );

  const { baseUrl } = useStores();
  const blobstoreUrl = `${baseUrl}/blobstore`;

  const {
    configuration,
    apriltagRigSolved,
    cameraRigSolved,
    solverStatus,
    rmse,
    stampBegin,
    stampEnd,
    eventLog,
  } = value;

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

      <Card title="Outputs">
        <Table bordered hover size="sm" responsive="md">
          <tbody>
            {apriltagRigSolved && (
              <JsonPopover
                trigger="hover"
                json={loadedApriltagRig || "loading..."}
                collapsed={1}
              >
                <tr>
                  <td> Apriltag Rig </td>
                  <td>
                    <Button
                      download
                      href={`${blobstoreUrl}/${apriltagRigSolved.path}`}
                    >
                      <Icon id="download" />
                    </Button>
                  </td>
                  <td>
                    <p>{apriltagRigSolved.path}</p>
                  </td>
                </tr>
              </JsonPopover>
            )}
            {cameraRigSolved && (
              <JsonPopover
                trigger="hover"
                json={loadedCameraRig || "loading..."}
                collapsed={3}
              >
                <tr>
                  <td> Camera Rig </td>
                  <td>
                    <Button
                      download
                      href={`${blobstoreUrl}/${cameraRigSolved.path}`}
                    >
                      <Icon id="download" />
                    </Button>
                  </td>
                  <td>
                    <p>{cameraRigSolved.path}</p>
                  </td>
                </tr>
              </JsonPopover>
            )}
          </tbody>
        </Table>
      </Card>
      {configuration && (
        <Card title="Configuration" collapsed>
          <CalibrateMultiViewApriltagRigConfigurationVisualizer.Element
            {...props}
            value={[0, configuration]}
          />
        </Card>
      )}
      {loadedInitial && (
        <Card title="Initial" collapsed>
          <MultiViewApriltagRigModelVisualizer.Element
            {...props}
            value={[0, loadedInitial]}
          />
        </Card>
      )}
      {loadedSolved && (
        <Card title="Solved" collapsed>
          <MultiViewApriltagRigModelVisualizer.Element
            {...props}
            value={[0, loadedSolved]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigResultVisualizer = {
  id: "CalibrateMultiViewApriltagRigResult",
  types: [
    "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigResult",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateMultiViewApriltagRigResultElement),
  Element: CalibrateMultiViewApriltagRigResultElement,
};
