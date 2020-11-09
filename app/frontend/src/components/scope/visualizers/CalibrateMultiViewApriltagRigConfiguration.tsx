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

import { Button, Table } from "react-bootstrap";
import { CalibrateMultiViewApriltagRigConfiguration } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig";
import { Card } from "./Card";
import { range } from "../../../utils/range";
import { Resource } from "../../../../genproto/farm_ng_proto/tractor/v1/resource";
import { uniquify } from "../../../utils/uniquify";
import { useFormState } from "../../../hooks/useFormState";
import { useState } from "react";
import Form from "./Form";

const CalibrateMultiViewApriltagRigConfigurationForm: React.FC<FormProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = (props) => {
  const [value, setValue] = useFormState(props);
  const [isAddingTagIds, setIsAddingTagIds] = useState(false);
  const [newTagIdRange, setNewTagIdRange] = useState({ start: 0, end: 0 });

  return (
    <>
      <Form.Group
        // TODO: Replace with resource browser
        label="Resource Path"
        value={value.videoDataset?.path}
        type="text"
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            videoDataset: Resource.fromPartial({
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
        value={value.rootCameraName}
        type="text"
        onChange={(e) => {
          const rootCameraName = e.target.value;
          setValue((v) => ({ ...v, rootCameraName }));
        }}
      />
    </>
  );
};

const CalibrateMultiViewApriltagRigConfigurationElement: React.FC<SingleElementVisualizerProps<
  CalibrateMultiViewApriltagRigConfiguration
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      TODO
    </Card>
  );
};

export const CalibrateMultiViewApriltagRigConfigurationVisualizer = {
  id: "CalibrateMultiViewApriltagRigConfiguration",
  types: [
    "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateMultiViewApriltagRigConfiguration"
  ],
  options: StandardComponentOptions,
  Component: StandardComponent(
    CalibrateMultiViewApriltagRigConfigurationElement
  ),
  Element: CalibrateMultiViewApriltagRigConfigurationElement,
  Form: CalibrateMultiViewApriltagRigConfigurationForm
};
