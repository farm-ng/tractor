import * as React from "react";
import { useState } from "react";
import RangeSlider from "react-bootstrap-range-slider";
import styles from "./Overlay.module.scss";
import { EventType } from "../../../registry/events";
import {
  SingleElementVisualizerProps,
  VisualizerProps
} from "../../../registry/visualization";
import { Icon } from "../../Icon";
import { Button } from "react-bootstrap";

export interface IProps<T extends EventType> {
  Element: React.FC<SingleElementVisualizerProps<T>>;
}
export type OverlayProps<T extends EventType> = IProps<T> & VisualizerProps<T>;

export const Overlay = <T extends EventType>(
  props: OverlayProps<T>
): React.ReactElement<OverlayProps<T>> | null => {
  const { Element, values } = props;

  const [index, setIndex] = useState(0);
  const [pinned, setPinned] = useState(true);

  const value = pinned ? values[values.length - 1] : values[index];
  if (!value) {
    // An external change (e.g. to the throttle) made the current index invalid.
    if (values.length > 0 && values[0]) {
      setIndex(0);
    }
    return null;
  }

  const togglePinned = (): void => {
    setPinned((prev) => {
      if (prev) {
        setIndex(values.length - 1);
      }
      return !prev;
    });
  };

  return (
    <div className={styles.overlay}>
      <div className={styles.overlayControls}>
        <RangeSlider
          className={styles.overlaySlider}
          value={pinned ? values.length - 1 : index}
          disabled={pinned}
          onChange={(_, v) => setIndex(v)}
          min={0}
          max={values.length - 1}
          step={1}
          tooltip={"off"}
        />
        <Button
          onClick={togglePinned}
          variant={pinned ? "light" : "outline-light"}
        >
          <Icon id="chevronBarRight" />
        </Button>
      </div>
      <Element value={value} {...props} />
    </div>
  );
};

export const OverlayVisualizerComponent = <T extends EventType>(
  Element: React.FC<SingleElementVisualizerProps<T>>
): React.FC<VisualizerProps<T>> => (props) => {
  return <Overlay Element={Element} {...props} />;
};

export const OverlayOptions = [];
