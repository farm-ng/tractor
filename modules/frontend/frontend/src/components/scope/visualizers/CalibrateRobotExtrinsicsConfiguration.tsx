/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { Card } from "./Card";
import { CalibrateRobotExtrinsicsConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_robot_extrinsics";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { CaptureRobotExtrinsicsDatasetResultVisualizer } from "./CaptureRobotExtrinsicsDatasetResult";
import { CaptureRobotExtrinsicsDatasetResult } from "@farm-ng/genproto-calibration/farm_ng/calibration/capture_robot_extrinsics_dataset";
import { ResourceVisualizer } from "./Resource";
// import { useStores } from "../../../hooks/useStores";

const CalibrateRobotExtrinsicsConfigurationForm: React.FC<FormProps<
  CalibrateRobotExtrinsicsConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);
  // const { httpResourceArchive } = useStores();

  // const loadedDataset = useFetchResource<CaptureRobotExtrinsicsDatasetResult>(
  //   value.dataset,
  //   httpResourceArchive
  // );


  return (
    <>
      <Form.Group
        label="Solve for Joint Offsets?"
        checked={value.jointOffsets}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const jointOffsets = Boolean(e.target.checked);
          setValue((v) => ({ ...v, jointOffsets }));
        }}
      />
      <Form.Group
        label="Disable reprojection images?"
        checked={value.disableReprojectionImages}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const disableReprojectionImages = Boolean(e.target.checked);
          setValue((v) => ({ ...v, disableReprojectionImages }));
        }}
      />
      <Form.Group
        label="Submit result?"
        checked={value.submit}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const submit = Boolean(e.target.checked);
          setValue((v) => ({ ...v, submit }));
        }}
      />

      <ResourceVisualizer.Form
        initialValue={Resource.fromPartial({
          path: value.dataset?.path,
          contentType:
            "application/json; type=type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetResult",
        })}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            dataset: updated,
          }))
        }
      />
    </>
  );
};

const CalibrateRobotExtrinsicsConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateRobotExtrinsicsConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const { jointOffsets, disableReprojectionImages, submit } = value;

  const dataset = useFetchResource<CaptureRobotExtrinsicsDatasetResult>(
    value.dataset,
    resources
  );

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Joint Offsets", jointOffsets],
            ["Disable Reprojection Images", disableReprojectionImages],
            ["Submit", submit],
          ]}
        />
      </Card>
      {dataset && (
        <Card title="Dataset">
          <CaptureRobotExtrinsicsDatasetResultVisualizer.Element
            {...props}
            value={[0, dataset]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateRobotExtrinsicsConfigurationVisualizer = {
  id: "CalibrateRobotExtrinsicsConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.CalibrateRobotExtrinsicsConfiguration",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    CalibrateRobotExtrinsicsConfigurationElement
  ),
  Element: CalibrateRobotExtrinsicsConfigurationElement,
  Form: CalibrateRobotExtrinsicsConfigurationForm,
};
