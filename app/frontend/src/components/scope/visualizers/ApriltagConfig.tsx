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
import { useEffect, useState } from "react";
import { simpleUniqueId } from "../../../utils/uniqueId";
import styles from "./ApriltagConfig.module.scss";

const ApriltagConfigForm: React.FC<FormProps<ApriltagConfig>> = (props) => {
  const [value, setValue] = useFormState(props);

  // React's reconciler requires a stable, unique key for each element in the list.
  // (https://reactjs.org/docs/lists-and-keys.html)
  // We can't use the element's index as its key, since we are adding/removing elements from the list.
  // And we can't use the element's tagId as its key, since that is editable by the user.
  // So we regenerate unique IDs for each element of the list everytime the length of the list changes.
  const [keyedTags, setKeyedTags] = useState<[string, TagConfig][]>();
  useEffect(() => {
    if (value.tagLibrary?.tags.length != keyedTags?.length) {
      console.log(
        "Recalculating keys: ",
        value.tagLibrary?.tags.length,
        keyedTags?.length
      );
      setKeyedTags(
        value.tagLibrary?.tags.map<[string, TagConfig]>((tag) => [
          simpleUniqueId(10),
          tag
        ]) || []
      );
    }
  }, [value]);

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
        </div>
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
