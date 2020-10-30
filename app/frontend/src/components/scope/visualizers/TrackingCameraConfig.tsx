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
import { useFormState } from "../../../hooks/useFormState";
import {
  CameraConfig,
  TrackingCameraConfig
} from "../../../../genproto/farm_ng_proto/tractor/v1/tracking_camera";
import { CameraConfigVisualizer } from "./CameraConfig";
import Form from "./Form";
import styles from "./TrackingCameraConfig.module.scss";
import { useStableKey } from "../../../hooks/useStableKey";

const TrackingCameraConfigForm: React.FC<FormProps<TrackingCameraConfig>> = (
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
                  [index]: updated
                })
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
                  ...(v.cameraConfigs || []).slice(index + 1)
                ]
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
              CameraConfig.fromPartial({})
            ]
          }))
        }
      />
    </>
  );
};

const TrackingCameraConfigElement: React.FC<SingleElementVisualizerProps<
  TrackingCameraConfig
>> = (props) => {
  const {
    value: [timestamp, value]
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

export const TrackingCameraConfigVisualizer = {
  id: "TrackingCameraConfig",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraConfig"],
  options: StandardComponentOptions,
  Component: StandardComponent(TrackingCameraConfigElement),
  Element: TrackingCameraConfigElement,
  Form: TrackingCameraConfigForm
};
