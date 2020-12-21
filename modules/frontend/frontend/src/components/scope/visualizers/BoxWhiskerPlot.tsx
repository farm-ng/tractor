import * as React from "react";
import { median, min, max, quantile } from "simple-statistics";
import uPlot from "uplot";
import {
  boxesPlugin,
  columnHighlightPlugin,
  legendAsTooltipPlugin,
} from "../../../utils/uplotPlugins";
import { Plot } from "./Plot";

interface IProps {
  title: string;
  data: { [key: string]: number[] };
  xAxisLabel: string;
}

export const BoxWhiskerPlot: React.FC<IProps> = ({
  data: inputData,
  title,
  xAxisLabel,
}) => {
  const [keys, values] = [Object.keys(inputData), Object.values(inputData)];
  const data = [
    new Array(keys.length).fill(0).map((_, i) => i),
    values.map((_) => median(_)),
    values.map((_) => quantile(_, 0.25)),
    values.map((_) => quantile(_, 0.75)),
    values.map((_) => min(_)),
    values.map((_) => max(_)),
  ];

  // Brittle, determined experimentally
  const xAxisHeight = 30 + max(keys.map((_) => _.length)) * 5;

  const options = {
    width: 800,
    height: 600,
    title: title,
    plugins: [boxesPlugin(), legendAsTooltipPlugin(), columnHighlightPlugin()],
    axes: [
      {
        rotate: -90,
        space: 10,
        size: xAxisHeight,
        grid: { show: false },
        values: (_: uPlot, vals: number[]) => vals.map((v) => keys[v]),
      },
    ],
    scales: { x: { distr: 2, time: false } as uPlot.Scale },
    series: [
      {
        label: xAxisLabel,
        value: (_: uPlot, i: number) => keys[i],
      },
      { label: "Median" },
      { label: "q1" },
      { label: "q3" },
      { label: "min" },
      { label: "max" },
    ],
  };

  return <Plot data={data} options={options} />;
};
