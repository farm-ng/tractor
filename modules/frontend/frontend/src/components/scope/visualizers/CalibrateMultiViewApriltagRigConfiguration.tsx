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
import { CalibrateMultiViewApriltagRigConfiguration } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_multi_view_apriltag_rig";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";
import { RepeatedIntForm } from "./RepeatedIntForm";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { CaptureVideoDatasetResultVisualizer } from "./CaptureVideoDatasetResult";
import { CaptureVideoDatasetResult } from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { ResourceVisualizer } from "./Resource";
import { useStores } from "../../../hooks/useStores";

const CalibrateMultiViewApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);
  const { httpResourceArchive } = useStores();

  const loadedVideoDataset = useFetchResource<CaptureVideoDatasetResult>(
    value.videoDataset,
    httpResourceArchive
  );

  return (
    <>
      <Form.Group
        label="Camera Rig Name"
        value={value.name}
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />

      <Form.Group
        label="Tag Rig Name"
        value={value.tagRigName}
        type="text"
        onChange={(e) => {
          const tagRigName = e.target.value;
          setValue((v) => ({ ...v, tagRigName }));
        }}
      />

      <ResourceVisualizer.Form
        initialValue={Resource.fromPartial({
          path: value.videoDataset?.path,
          contentType:
            "application/json; type=type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult",
        })}
        onChange={(updated) =>
          setValue((v) => ({
            ...v,
            videoDataset: updated,
          }))
        }
      />

      {loadedVideoDataset && (
        <>
          <Form.Group
            label="Root Camera Name"
            value={value.rootCameraName}
            as="select"
            onChange={(e) => {
              const rootCameraName = e.target.value;
              setValue((v) => ({ ...v, rootCameraName }));
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

          <h6>Tag IDs</h6>
          <RepeatedIntForm
            initialValue={loadedVideoDataset.perTagIdNumFrames.map(
              (_) => _.tagId
            )}
            onChange={(updated) =>
              setValue((v) => ({
                ...v,
                tagIds: updated,
              }))
            }
          />

          <Form.Group
            label="Root Tag ID"
            value={value.rootTagId}
            as="select"
            onChange={(e) => {
              const rootTagId = parseInt(e.target.value);
              setValue((v) => ({ ...v, rootTagId }));
            }}
          >
            {[
              <option key={"SELECT"} value={""}>
                {"SELECT"}
              </option>,
              ...loadedVideoDataset.perTagIdNumFrames
                .map((entry) => entry.tagId)
                .map((tagId) => {
                  return (
                    <option key={tagId} value={tagId}>
                      {tagId}
                    </option>
                  );
                }),
            ]}
          </Form.Group>

          <Form.Group
            label="Filter stable tags?"
            checked={value.filterStableTags}
            type="checkbox"
            onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
              const filterStableTags = Boolean(e.target.checked);
              setValue((v) => ({ ...v, filterStableTags }));
            }}
          />
        </>
      )}
    </>
  );
};

const CalibrateMultiViewApriltagRigConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const { tagIds, rootTagId, rootCameraName, name, tagRigName } = value;

  const videoDataset = useFetchResource<CaptureVideoDatasetResult>(
    value.videoDataset,
    resources
  );

  return (
    <Card timestamp={timestamp} json={value}>
      <Card title="Summary">
        <KeyValueTable
          records={[
            ["Name", name],
            ["Tag IDs", (tagIds || []).join(", ")],
            ["Root Tag ID", rootTagId],
            ["Root Camera Name", rootCameraName],
            ["Tag Rig Name", tagRigName],
          ]}
        />
      </Card>
      {videoDataset && (
        <Card title="Video Dataset">
          <CaptureVideoDatasetResultVisualizer.Element
            {...props}
            value={[0, videoDataset]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigConfigurationVisualizer = {
  id: "CalibrateMultiViewApriltagRigConfiguration",
  types: [
    "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigConfiguration",
  ],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(
    CalibrateMultiViewApriltagRigConfigurationElement
  ),
  Element: CalibrateMultiViewApriltagRigConfigurationElement,
  Form: CalibrateMultiViewApriltagRigConfigurationForm,
};
