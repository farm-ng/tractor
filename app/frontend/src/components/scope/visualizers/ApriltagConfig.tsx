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
import { Card } from "./Card";
import {
  ApriltagConfig,
  TagConfig
} from "../../../../genproto/farm_ng_proto/tractor/v1/apriltag";
import { KeyValueTable } from "./KeyValueTable";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { TagConfigVisualizer } from "./TagConfig";

const ApriltagConfigForm: React.FC<FormProps<ApriltagConfig>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <h6>Tag IDs</h6>
      {value.tagLibrary?.tags.map((tag, index) => (
        <React.Fragment key={`${tag.id}_${index}`}>
          <TagConfigVisualizer.Form
            initialValue={tag}
            onChange={(updated) =>
              setValue((v) => ({
                ...v,
                tagLibrary: {
                  ...v.tagLibrary,
                  tags: Object.assign([...(v.tagLibrary?.tags || [])], {
                    [index]: updated
                  })
                }
              }))
            }
          />

          <Form.ButtonGroup
            buttonText="X"
            onClick={() =>
              setValue((v) => ({
                ...v,
                tagLibrary: {
                  ...v.tagLibrary,
                  tags: [
                    ...(v.tagLibrary?.tags || []).slice(0, index),
                    ...(v.tagLibrary?.tags || []).slice(index + 1)
                  ]
                }
              }))
            }
          />
        </React.Fragment>
      ))}

      <Form.ButtonGroup
        buttonText="+"
        onClick={() =>
          setValue((v) => ({
            ...v,
            tagLibrary: {
              ...v.tagLibrary,
              tags: [...(v.tagLibrary?.tags || []), TagConfig.fromPartial({})]
            }
          }))
        }
      />
    </>
  );
};

const ApriltagConfigElement: React.FC<SingleElementVisualizerProps<
  ApriltagConfig
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      {value.tagLibrary && (
        <KeyValueTable
          headers={["ID", "Size (m)"]}
          records={value.tagLibrary.tags.map<[string, unknown]>((tag) => [
            tag.id.toString(),
            tag.size
          ])}
        />
      )}
    </Card>
  );
};

export const ApriltagConfigVisualizer = {
  id: "ApriltagConfig",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagConfig"],
  options: StandardComponentOptions,
  Component: StandardComponent(ApriltagConfigElement),
  Element: ApriltagConfigElement,
  Form: ApriltagConfigForm
};
