/* eslint-disable no-console */
import * as React from "react";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { CalibrateIntrinsicsResult } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_intrinsics";
import { Card } from "./Card";
import { CalibrateIntrinsicsConfigurationVisualizer } from "./CalibrateIntrinsicsConfiguration";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { IntrinsicModel } from "@farm-ng/genproto-calibration/farm_ng/calibration/intrinsic_model";
import { KeyValueTable } from "./KeyValueTable";
import { solverStatusToJSON } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { IntrinsicModelVisualizer } from "./IntrinsicModel";
import { useStores } from "../../../hooks/useStores";
import { JsonPopover } from "../../JsonPopover";
import { Icon } from "../../Icon";
import { Button, Table } from "react-bootstrap";
import { CameraModel } from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";

const CalibrateIntrinsicsResultElement: React.FC<SingleElementVisualizerProps<
  CalibrateIntrinsicsResult
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const { baseUrl } = useStores();
  const blobstoreUrl = `${baseUrl}/blobstore`;

  const loadedIntrinsicsSolved = useFetchResource<IntrinsicModel>(
    value.intrinsicsSolved,
    resources
  );
  const loadedCameraModel = useFetchResource<CameraModel>(
    value.cameraModel,
    resources
  );

  const {
    configuration,
    solverStatus,
    cameraModel,
    rmse,
    stampBegin,
    stampEnd,
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
          ]}
        />
      </Card>
      <Card title="Outputs">
        <Table bordered hover size="sm" responsive="md">
          <tbody>
            {cameraModel && (
              <JsonPopover
                trigger="hover"
                json={loadedCameraModel || "loading..."}
                collapsed={1}
              >
                <tr>
                  <td> Camera Model </td>
                  <td>
                    <Button
                      download
                      href={`${blobstoreUrl}/${cameraModel.path}`}
                    >
                      <Icon id="download" />
                    </Button>
                  </td>
                  <td>
                    <p>{cameraModel.path}</p>
                  </td>
                </tr>
              </JsonPopover>
            )}
          </tbody>
        </Table>
      </Card>
      {configuration && (
        <Card title="Configuration">
          {
            <CalibrateIntrinsicsConfigurationVisualizer.Element
              {...props}
              value={[0, configuration]}
            />
          }
        </Card>
      )}
      {loadedIntrinsicsSolved && (
        <Card title="Details">
          <IntrinsicModelVisualizer.Element
            {...props}
            value={[0, loadedIntrinsicsSolved]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateIntrinsicsResultVisualizer = {
  id: "CalibrateIntrinsicsResult",
  types: ["type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsResult"],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateIntrinsicsResultElement),
  Element: CalibrateIntrinsicsResultElement,
};
