import * as React from "react";
import { EventType } from "../../../registry/events";
import {
  SingleElementVisualizerProps,
  VisualizerProps
} from "../../../registry/visualization";
import { Grid } from "./Grid";
import { Overlay } from "./Overlay";

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

export const StandardComponentOptions = [
  { label: "view", options: ["overlay", "grid"] }
];
