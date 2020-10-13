import * as React from "react";
import { useEffect, useState } from "react";
import { useResizeObserver } from "../../../hooks/useResizeObserver";
import uPlot from "uplot";
import styles from "./Plot.module.scss";
import "uplot/dist/uPlot.min.css";

interface IProps {
  data: uPlot.AlignedData;
  options: uPlot.Options;
}

export const Plot: React.FC<IProps> = ({ data, options }) => {
  const [uPlotInstance, setUPlotInstance] = useState<uPlot>();

  // Handle resizing
  const [container, containerRef, resizeObservation] = useResizeObserver();
  useEffect(() => {
    if (resizeObservation && uPlotInstance) {
      uPlotInstance.setSize({
        width: Math.floor(resizeObservation.contentRect.width),
        height: Math.floor(resizeObservation.contentRect.height) - 25
      });
    }
  }, [resizeObservation, uPlotInstance]);

  // On mount, or if options change, instantiate a new uPlot instance
  useEffect(() => {
    if (!container) {
      return;
    }
    setUPlotInstance(
      (existing) => existing || new uPlot(options, data, container)
    );
  }, [container, options]);

  // On data change, just update the data
  useEffect(() => {
    if (!uPlotInstance) {
      return;
    }
    uPlotInstance.setData(data);
    uPlotInstance.redraw();
  }, [data]);

  return (
    <div
      id={`plot-${Math.random()}`}
      className={styles.plot}
      ref={containerRef}
    ></div>
  );
};
