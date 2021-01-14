import * as React from "react";
import { useMemo } from "react";
import { EventType } from "../../../registry/events";
import { VisualizerProps } from "../../../registry/visualization";
import { colorGenerator } from "../../../utils/colorGenerator";
import { Plot } from "./Plot";

const TimeSkewComponent: React.FC<VisualizerProps<EventType>> = ({
  values,
}) => {
  const plotData = [values.map((_, i) => i), values.map(([t, _]) => t / 1000)];

  const colors = colorGenerator();
  const plotOptions = useMemo(
    () => ({
      width: 800,
      height: 600,
      series: [
        { show: false },
        { label: "t", stroke: colors.next().value, width: 1 },
      ],
      scales: { x: { time: false }, y: { time: true } },
      axes: [{}, { size: 100 }],
    }),
    []
  );

  return <Plot data={plotData} options={plotOptions} />;
};

export const TimeSkewVisualizer = {
  id: "TimeSkew",
  types: "*" as const,
  options: [],
  MultiElement: TimeSkewComponent,
};
