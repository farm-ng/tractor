import {
  ODriveAxis,
  oDriveAxis_StateToJSON as StateToJSON,
  oDriveAxis_ErrorToJSON as ErrorToJSON,
} from "@farm-ng/genproto-motors/farm_ng/motors/motor";
import * as React from "react";
import { VisualizerProps } from "../../../registry/visualization";
import { colorGenerator } from "../../../utils/colorGenerator";
import { KeyValueTable } from "./KeyValueTable";
import { Plot } from "./Plot";

const ODriveAxisComponent: React.FC<VisualizerProps<ODriveAxis>> = ({
  values,
}) => {
  const plotData = [
    values.map(([t, _]) => t / 1000),
    // TODO(isherman): How to visually display undefined values?
    values.map(([_, v]) => v.encoderPositionEstimate || 0),
    values.map(([_, v]) => v.encoderVelocityEstimate || 0),
    values.map(([_, v]) => v.inputVelocity || 0),
    values.map(([_, v]) => v.currentState || 0),
  ];

  const colors = colorGenerator();
  const plotOptions = {
    width: 800,
    height: 600,
    series: [
      {
        show: false,
      },
      ...["pos_est", "vel_est", "vel_cmd"].map((label) => ({
        label,
        stroke: colors.next().value,
      })),
      {
        label: "state",
        value: (_: unknown, v: number) => StateToJSON(v),
      },
    ],
  };

  const latestValue = values[values.length - 1][1];

  return (
    <>
      <Plot data={plotData} options={plotOptions} />
      <KeyValueTable
        records={[
          ["Latest State", StateToJSON(latestValue.currentState)],
          ...latestValue.error.map<[string, unknown]>((err, i) => [
            `Latest Error #${i}`,
            ErrorToJSON(err),
          ]),
        ]}
      />
    </>
  );
};

export const ODriveAxisVisualizer = {
  id: "ODriveAxis",
  types: ["type.googleapis.com/farm_ng.motors.ODriveAxis"],
  options: [],
  Component: ODriveAxisComponent,
};
