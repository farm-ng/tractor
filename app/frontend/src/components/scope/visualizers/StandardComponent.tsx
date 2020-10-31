import * as React from "react";
import { EventType } from "../../../registry/events";
import {
  SingleElementVisualizerProps,
  VisualizerProps
} from "../../../registry/visualization";
import { Card } from "./Card";
import { Grid } from "./Grid";
import { Overlay } from "./Overlay";
import { Scene } from "./Scene";

export const StandardComponent = <T extends EventType>(
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

export const Standard3DElement = <T extends EventType>(
  Element: React.FC<SingleElementVisualizerProps<T>>
): React.FC<SingleElementVisualizerProps<T>> => ({
  value: [timestamp, value]
}) => {
  return (
    <Card timestamp={timestamp} json={value}>
      <Scene>
        <Element value={[timestamp, value]} />
      </Scene>
    </Card>
  );
};

export const Standard3DComponent = <T extends EventType>(
  Element: React.FC<SingleElementVisualizerProps<T>>
): React.FC<VisualizerProps<T>> => (props) => {
  return <Overlay Element={Element} {...props} />;
};

export const Standard3DComponentOptions = [];

export const StandardComponentOptions = [
  { label: "view", options: ["overlay", "grid"] }
];
