import * as React from "react";
import { useState } from "react";
import RangeSlider from "react-bootstrap-range-slider";
import { EventType } from "../../../registry/events";
import {
  SingleElementVisualizerProps,
  VisualizerProps
} from "../../../registry/visualization";
import styles from "./Grid.module.scss";

const defaultPageSize = 12;

interface IProps<T extends EventType> {
  Element: React.FC<SingleElementVisualizerProps<T>>;
  pageSize?: number;
}
export type GridProps<T extends EventType> = IProps<T> & VisualizerProps<T>;

export const Grid = <T extends EventType>(
  props: GridProps<T>
): React.ReactElement<GridProps<T>> => {
  const { Element, values } = props;

  const [page, setPage] = useState(0);
  const [pageSize, setPageSize] = useState(props.pageSize || defaultPageSize);
  const numPages = values.length / pageSize;

  return (
    <div>
      <div className={styles.slider}>
        <span>Page</span>
        <RangeSlider
          value={page}
          onChange={(_, v) => setPage(v)}
          min={0}
          max={numPages}
          step={1}
        />
      </div>
      <div className={styles.slider}>
        <span>Page Size</span>
        <RangeSlider
          value={pageSize}
          onChange={(_, v) => setPageSize(v)}
          min={1}
          max={24}
          step={1}
        />
      </div>
      <div className={styles.grid}>
        {values.slice(page * pageSize, (page + 1) * pageSize).map((v) => (
          <div key={v[0]} className={styles.gridItem}>
            <Element value={v} {...props} />
          </div>
        ))}
      </div>
    </div>
  );
};
