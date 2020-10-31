/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps
} from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { CalibrateApriltagRigConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_apriltag_rig";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { KeyValueTable } from "./KeyValueTable";
import { Card } from "./Card";
import { CaptureCalibrationDatasetResult } from "../../../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { Resource } from "../../../../genproto/farm_ng_proto/tractor/v1/resource";
import { CaptureCalibrationDatasetResultVisualizer } from "./CaptureCalibrationDatasetResult";
import { useState } from "react";
import { range } from "../../../utils/range";
import { Button, Table } from "react-bootstrap";
import { uniquify } from "../../../utils/uniquify";

CaptureCalibrationDatasetResultVisualizer.Element;

const CalibrateApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateApriltagRigConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);
  const [isAddingTagIds, setIsAddingTagIds] = useState(false);
  const [newTagIdRange, setNewTagIdRange] = useState({ start: 0, end: 0 });

  return (
    <>
      <Form.Group
        // TODO: Replace with resource browser
        label="Resource Path"
        value={value.calibrationDataset?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            calibrationDataset: Resource.fromPartial({
              path,
              contentType:
                "application/json; type=type.googleapis.com/farm_ng_proto.tractor.v1.CaptureVideoDatasetResult"
            })
          }));
        }}
      />
      <h6>Tag IDs</h6>
      <Table striped bordered size="sm" responsive="md">
        {value.tagIds.map((tagId, index) => (
          <tr key={index}>
            <td>{tagId}</td>
            <td>
              <Button
                onClick={() =>
                  setValue((v) => ({
                    ...v,
                    tagIds: [
                      ...v.tagIds.slice(0, index),
                      ...v.tagIds.slice(index + 1)
                    ]
                  }))
                }
              >
                X
              </Button>
            </td>
          </tr>
        ))}
      </Table>

      {!isAddingTagIds && (
        <Form.ButtonGroup
          buttonText="+"
          onClick={() => setIsAddingTagIds(true)}
        />
      )}

      {isAddingTagIds && (
        <>
          <Form.Group
            label={`Range Start`}
            value={newTagIdRange.start}
            type="number"
            onChange={(e) => {
              const start = parseInt(e.target.value);
              setNewTagIdRange((r) => ({
                ...r,
                start
              }));
            }}
          />

          <Form.Group
            label={`Range End (inclusive)`}
            value={newTagIdRange.end}
            type="number"
            onChange={(e) => {
              const end = parseInt(e.target.value);
              setNewTagIdRange((r) => ({
                ...r,
                end
              }));
            }}
          />

          <Form.ButtonGroup
            buttonText="✓"
            onClick={() => {
              console.log(
                uniquify(
                  [...range(newTagIdRange.start, newTagIdRange.end + 1)].sort()
                )
              );
              setValue((v) => ({
                ...v,
                tagIds: uniquify(
                  [
                    ...v.tagIds,
                    ...range(newTagIdRange.start, newTagIdRange.end + 1)
                  ].sort((a, b) => a - b)
                )
              }));
              setIsAddingTagIds(false);
            }}
          />

          <Form.ButtonGroup
            buttonText="×"
            onClick={() => setIsAddingTagIds(false)}
          />
        </>
      )}

      <Form.Group
        label="Name"
        value={value.name}
        type="text"
        onChange={(e) => {
          const name = e.target.value;
          setValue((v) => ({ ...v, name }));
        }}
      />

      <Form.Group
        label="Root Tag ID"
        value={value.rootTagId}
        type="number"
        onChange={(e) => {
          const rootTagId = parseInt(e.target.value);
          setValue((v) => ({ ...v, rootTagId }));
        }}
      />

      <Form.Group
        label="Filter stable tags?"
        checked={value.filterStableTags}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const filterStableTags = Boolean(e.target.checked);
          setValue((v) => ({ ...v, filterStableTags }));
        }}
      />

      <Form.Group
        label="Camera Name"
        value={value.cameraName}
        type="text"
        onChange={(e) => {
          const cameraName = e.target.value;
          setValue((v) => ({ ...v, cameraName }));
        }}
      />
    </>
  );
};

const CalibrateApriltagRigConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateApriltagRigConfiguration
>> = (props) => {
  const {
    value: [timestamp, value],
    resources
  } = props;

  const { tagIds, rootTagId, name, filterStableTags, cameraName } = value;

  const calibrationDataset = useFetchResource<CaptureCalibrationDatasetResult>(
    value.calibrationDataset,
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
            ["Filter Stable Tags?", filterStableTags],
            ["Camera Name", cameraName]
          ]}
        />
      </Card>
      {calibrationDataset && (
        <Card title="Calibration Dataset">
          <CaptureCalibrationDatasetResultVisualizer.Element
            {...props}
            value={[0, calibrationDataset]}
          />
        </Card>
      )}
    </Card>
  );
};

export const CalibrateApriltagRigConfigurationVisualizer = {
  id: "CalibrateApriltagRigConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(CalibrateApriltagRigConfigurationElement),
  Element: CalibrateApriltagRigConfigurationElement,
  Form: CalibrateApriltagRigConfigurationForm
};
