import * as React from "react";
import { EventType } from "../../../registry/events";
import {
  SingleElementVisualizerProps,
  VisualizerProps,
} from "../../../registry/visualization";
import { Card } from "./Card";
import { Grid } from "./Grid";
import { Overlay } from "./Overlay";
import { Scene } from "./Scene";

export const StandardMultiElement = <T extends EventType>(
  Element: React.FC<SingleElementVisualizerProps<T>>
): React.FC<VisualizerProps<T>> => (props) => {
  const view = props.options[0].value;
  return (
    <>
      {view === "grid" && <Grid {...props} Element={Element} />}
      {view === "overlay" && <Overlay {...props} Element={Element} />}
    </>
  );
};

export const StandardElement3D = <T extends EventType>(
  Element: React.FC<SingleElementVisualizerProps<T>>
): React.FC<SingleElementVisualizerProps<T>> => ({
  value: [timestamp, value],
}) => {
  return (
    <Card timestamp={timestamp} json={value}>
      <Scene>
        <Element value={[timestamp, value]} />
      </Scene>
    </Card>
  );
};

export const StandardMultiElement3D = <T extends EventType>(
  Element: React.FC<SingleElementVisualizerProps<T>>
): React.FC<VisualizerProps<T>> => (props) => {
  return <Overlay Element={Element} {...props} />;
};

export const StandardMultiElement3DOptions = [];

export const StandardMultiElementOptions = [
  { label: "view", options: ["overlay", "grid"] },
];
