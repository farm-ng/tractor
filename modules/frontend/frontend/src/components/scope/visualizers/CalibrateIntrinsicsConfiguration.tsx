/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { CalibrateIntrinsicsConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_intrinsics";
import { Card } from "./Card";
import { KeyValueTable } from "./KeyValueTable";
import { cameraModel_DistortionModelToJSON } from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";

const CalibrateIntrinsicsConfigurationForm: React.FC<FormProps<
  CalibrateIntrinsicsConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  const { videoDataset, cameraName, filterStableTags } = value;

  return (
    <>
      <Form.Group
        // TODO: Replace with resource browser
        label="Video Dataset"
        value={videoDataset?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            videoDataset: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng.perception.CreateVideoDatasetResult",
            }),
          }));
        }}
      />

      <Form.Group
        label="Camera Name"
        value={cameraName}
        type="text"
        onChange={(e) => {
          const cameraName = e.target.value;
          setValue((v) => ({ ...v, cameraName }));
        }}
      />

      <Form.Group
        label="Filter Stable Tags?"
        checked={filterStableTags}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const filterStableTags = Boolean(e.target.checked);
          setValue((v) => ({ ...v, filterStableTags }));
        }}
      />
    </>
  );
};

const CalibrateIntrinsicsConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateIntrinsicsConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  const { cameraName, filterStableTags, videoDataset, distortionModel } = value;

  return (
    <Card timestamp={timestamp} json={value}>
      <KeyValueTable
        records={[
          ["Video Dataset", videoDataset?.path],
          [
            "Distortion Model",
            cameraModel_DistortionModelToJSON(distortionModel),
          ],
          ["Camera Name", cameraName],
          ["Filter Stable Tags?", filterStableTags],
        ]}
      />
    </Card>
  );
};

export const CalibrateIntrinsicsConfigurationVisualizer = {
  id: "CalibrateIntrinsicsConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsConfiguration",
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateIntrinsicsConfigurationElement),
  Element: CalibrateIntrinsicsConfigurationElement,
  Form: CalibrateIntrinsicsConfigurationForm,
};
