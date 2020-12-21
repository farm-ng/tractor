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
import {
  CameraModel_DistortionModel,
  cameraModel_DistortionModelToJSON,
} from "@farm-ng/genproto-perception/farm_ng/perception/camera_model";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { ResourceVisualizer } from "./Resource";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { CreateVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/create_video_dataset";
import { useStores } from "../../../hooks/useStores";
import { enumNumericKeys } from "../../../utils/enum";

const CalibrateIntrinsicsConfigurationForm: React.FC<FormProps<
  CalibrateIntrinsicsConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);

  const { cameraName, filterStableTags } = value;
  const { httpResourceArchive } = useStores();

  const loadedVideoDataset = useFetchResource<CreateVideoDatasetResult>(
    value.videoDataset,
    httpResourceArchive
  );

  return (
    <>
      <ResourceVisualizer.Form
        initialValue={Resource.fromPartial({
          path: value.videoDataset?.path,
          contentType:
            "application/json; type=type.googleapis.com/farm_ng.perception.CreateVideoDatasetResult",
        })}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            videoDataset: updated,
          }))
        }
      />

      <Form.Group
        label="Distortion Model"
        value={value.distortionModel}
        as="select"
        onChange={(e) => {
          const distortionModel = parseInt(e.target.value);
          setValue((v) => ({ ...v, distortionModel }));
        }}
      >
        {enumNumericKeys(CameraModel_DistortionModel)
          .filter((k) => k >= 0)
          .map((k) => {
            return (
              <option key={k} value={k}>
                {cameraModel_DistortionModelToJSON(k)}
              </option>
            );
          })}
      </Form.Group>

      {loadedVideoDataset && (
        <Form.Group
          label="Camera Name"
          value={cameraName}
          as="select"
          onChange={(e) => {
            const cameraName = e.target.value;
            setValue((v) => ({ ...v, cameraName }));
          }}
        >
          {[
            <option key={"SELECT"} value={""}>
              {"SELECT"}
            </option>,
            ...loadedVideoDataset.perCameraNumFrames
              .map((entry) => entry.cameraName)
              .map((cameraName) => {
                return (
                  <option key={cameraName} value={cameraName}>
                    {cameraName}
                  </option>
                );
              }),
          ]}
        </Form.Group>
      )}

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
