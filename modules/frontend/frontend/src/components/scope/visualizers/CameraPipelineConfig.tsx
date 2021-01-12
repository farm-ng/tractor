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
import { useFormState } from "../../../hooks/useFormState";
import {
  CameraConfig,
  CameraPipelineConfig,
} from "@farm-ng/genproto-perception/farm_ng/perception/camera_pipeline";
import { CameraConfigVisualizer } from "./CameraConfig";
import Form from "./Form";
import styles from "./CameraPipelineConfig.module.scss";
import { useStableKey } from "../../../hooks/useStableKey";

const CameraPipelineConfigForm: React.FC<FormProps<CameraPipelineConfig>> = (
  props
) => {
  const [value, setValue] = useFormState(props);
  const keyedConfigs = useStableKey(value.cameraConfigs);

  return (
    <>
      {keyedConfigs?.map(([key, cameraConfig], index) => (
        <div className={styles.row} key={key}>
          <CameraConfigVisualizer.Form
            initialValue={cameraConfig}
            onChange={(updated) =>
              setValue((v) => ({
                ...v,
                cameraConfigs: Object.assign([...v.cameraConfigs], {
                  [index]: updated,
                }),
              }))
            }
          />
          <Form.ButtonGroup
            buttonText="X"
            onClick={() =>
              setValue((v) => ({
                ...v,
                cameraConfigs: [
                  ...(v.cameraConfigs || []).slice(0, index),
                  ...(v.cameraConfigs || []).slice(index + 1),
                ],
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
            cameraConfigs: [
              ...(v.cameraConfigs || []),
              CameraConfig.fromPartial({}),
            ],
          }))
        }
      />
    </>
  );
};

const CameraPipelineConfigElement: React.FC<SingleElementVisualizerProps<
  CameraPipelineConfig
>> = (props) => {
  const {
    value: [timestamp, value],
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      {value.cameraConfigs?.map((cameraConfig) => {
        return (
          <Card key={cameraConfig.name}>
            <CameraConfigVisualizer.Element
              {...props}
              value={[0, cameraConfig]}
            />
          </Card>
        );
      })}
    </Card>
  );
};

export const CameraPipelineConfigVisualizer = {
  id: "CameraPipelineConfig",
  types: ["type.googleapis.com/farm_ng.perception.CameraPipelineConfig"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(CameraPipelineConfigElement),
  Element: CameraPipelineConfigElement,
  Form: CameraPipelineConfigForm,
};
