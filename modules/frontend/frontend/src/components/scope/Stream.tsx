import { useObserver } from "mobx-react-lite";
import * as React from "react";
import { TimestampedEventVector } from "../../types/common";
import { useStores } from "../../hooks/useStores";
import styles from "./Stream.module.scss";
import { Panel } from "../../stores/VisualizationStore";

interface IProps {
  panel: Panel;
  name: string;
  values: TimestampedEventVector;
}

const filter = (
  values: TimestampedEventVector,
  start: Date | null,
  end: Date | null,
  throttle: number
): TimestampedEventVector =>
  values.reduce<TimestampedEventVector>((acc, [t, v]) => {
    if ((start && t < start.getTime()) || (end && t > end.getTime())) {
      return acc;
    }
    if (acc.length > 0 && t - acc[acc.length - 1][0] <= throttle) {
      return acc;
    }
    acc.push([t, v]);
    return acc;
  }, []);

export const Stream: React.FC<IProps> = ({ panel, name, values }) => {
  const { visualizationStore: store } = useStores();

  return useObserver(() => {
    const { visualizer, options } = panel;

    const filteredValues = filter(
      values,
      store.bufferRangeStartDate,
      store.bufferRangeEndDate,
      store.bufferThrottle
    );

    return (
      <div className={styles.stream}>
        <h4>{name}</h4>
        {React.createElement(visualizer.Component, {
          values: filteredValues,
          options,
          resources: store.resourceArchive
        })}
      </div>
    );
  });
};
