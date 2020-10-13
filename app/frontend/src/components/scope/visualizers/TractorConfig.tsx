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
import { TractorConfig } from "../../../../genproto/farm_ng_proto/tractor/v1/tractor";
import { KeyValueTable } from "./KeyValueTable";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";

const TractorConfigForm: React.FC<FormProps<TractorConfig>> = (props) => {
  const [value, setValue] = useFormState(props);

  return (
    <>
      <Form.Group
        label="Wheel Baseline"
        value={value.wheelBaseline}
        type="number"
        onChange={(e) => {
          const wheelBaseline = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            wheelBaseline
          }));
        }}
      />

      <Form.Group
        label="Wheel Radius"
        value={value.wheelRadius}
        type="number"
        onChange={(e) => {
          const wheelRadius = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            wheelRadius
          }));
        }}
      />

      <Form.Group
        label="Wheel Radius Left"
        value={value.wheelRadiusLeft}
        type="number"
        onChange={(e) => {
          const wheelRadiusLeft = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            wheelRadiusLeft
          }));
        }}
      />

      <Form.Group
        label="Wheel Radius Right"
        value={value.wheelRadiusRight}
        type="number"
        onChange={(e) => {
          const wheelRadiusRight = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            wheelRadiusRight
          }));
        }}
      />

      <Form.Group
        label="Hub Motor Gear Ratio"
        value={value.hubMotorGearRatio}
        type="number"
        onChange={(e) => {
          const hubMotorGearRatio = parseFloat(e.target.value);
          setValue((v) => ({
            ...v,
            hubMotorGearRatio
          }));
        }}
      />

      <Form.Group
        label="Hub Motor Poll Pairs"
        value={value.hubMotorPollPairs}
        type="number"
        onChange={(e) => {
          const hubMotorPollPairs = parseInt(e.target.value);
          setValue((v) => ({
            ...v,
            hubMotorPollPairs
          }));
        }}
      />

      {value.basePosesSensor.map((basePoseSensor, i) => (
        <NamedSE3PoseVisualizer.Form
          key={basePoseSensor.frameA + basePoseSensor.frameB}
          initialValue={basePoseSensor}
          onChange={(updated) =>
            setValue((v) => ({
              ...v,
              basePosesSensor: Object.assign([...v.basePosesSensor], {
                [i]: updated
              })
            }))
          }
        />
      ))}
    </>
  );
};

const TractorConfigElement: React.FC<SingleElementVisualizerProps<
  TractorConfig
>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;

  return (
    <Card timestamp={timestamp} json={value}>
      <KeyValueTable
        records={[
          ["Wheel Baseline", value.wheelBaseline],
          ["Wheel Radius", value.wheelRadius],
          ["Wheel Radius Left", value.wheelRadiusLeft],
          ["Wheel Radius Right", value.wheelRadiusRight],
          ["Hub Motor Gear Ratio", value.hubMotorGearRatio],
          ["Hub Motor Poll Pairs", value.hubMotorPollPairs]
        ]}
      />

      {value.basePosesSensor.map((basePoseSensor) => {
        const title = `${basePoseSensor.frameA}_pose_${basePoseSensor.frameB}`;
        return (
          <Card key={title} title={title}>
            <NamedSE3PoseVisualizer.Element
              {...props}
              value={[0, basePoseSensor]}
            />
          </Card>
        );
      })}
    </Card>
  );
};

export const TractorConfigVisualizer = {
  id: "TractorConfig",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.TractorConfig"],
  options: StandardComponentOptions,
  Component: StandardComponent(TractorConfigElement),
  Element: TractorConfigElement,
  Form: TractorConfigForm
};
