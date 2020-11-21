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
import { Card } from "./Card";
import {
  ApriltagConfig,
  TagConfig,
} from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import { KeyValueTable } from "./KeyValueTable";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { TagConfigVisualizer } from "./TagConfig";
import styles from "./ApriltagConfig.module.scss";
import { useStableKey } from "../../../hooks/useStableKey";

const ApriltagConfigForm: React.FC<FormProps<ApriltagConfig>> = (props) => {
  const [value, setValue] = useFormState(props);
  const keyedTags = useStableKey(value.tagLibrary?.tags);

  return (
    <>
      {keyedTags?.map(([tagKey, tag], index) => (
        <div className={styles.row} key={tagKey}>
          <TagConfigVisualizer.Form
            initialValue={tag}
            onChange={(updated) =>
              setValue((v) => ({
                ...v,
                tagLibrary: {
                  ...v.tagLibrary,
                  tags: Object.assign([...(v.tagLibrary?.tags || [])], {
                    [index]: updated,
                  }),
                },
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
                    ...(v.tagLibrary?.tags || []).slice(index + 1),
                  ],
                },
              }))
            }
          />
        </div>
      ))}

      <Form.ButtonGroup
        buttonText="+"
        onClick={() =>
          setValue((v) => ({
            ...v,
            tagLibrary: {
              ...v.tagLibrary,
              tags: [...(v.tagLibrary?.tags || []), TagConfig.fromPartial({})],
            },
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
    value: [timestamp, value],
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      {value.tagLibrary && (
        <KeyValueTable
          headers={["ID", "Size (m)"]}
          records={value.tagLibrary.tags
            .sort((a, b) => a.id - b.id)
            .map<[string, unknown]>((tag) => [tag.id.toString(), tag.size])}
        />
      )}
    </Card>
  );
};

export const ApriltagConfigVisualizer = {
  id: "ApriltagConfig",
  types: ["type.googleapis.com/farm_ng.perception.ApriltagConfig"],
  options: StandardComponentOptions,
  Component: StandardComponent(ApriltagConfigElement),
  Element: ApriltagConfigElement,
  Form: ApriltagConfigForm,
};
