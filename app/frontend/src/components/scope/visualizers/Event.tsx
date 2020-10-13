/* eslint-disable no-console */
import * as React from "react";
import { Card } from "./Card";
import {
  SingleElementVisualizerProps,
  visualizersForEventType
} from "../../../registry/visualization";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { Event as BusEvent } from "../../../../genproto/farm_ng_proto/tractor/v1/io";
import { EventTypeId } from "../../../registry/events";
import { decodeAnyEvent } from "../../../models/decodeAnyEvent";

const EventElement: React.FC<SingleElementVisualizerProps<BusEvent>> = (
  props
) => {
  const {
    value: [timestamp, value]
  } = props;

  if (!value.data) {
    return null;
  }
  const visualizer = visualizersForEventType(
    value.data.typeUrl as EventTypeId
  )[0];

  if (!visualizer || !visualizer.Element) {
    return null;
  }

  const data = decodeAnyEvent(value);
  if (!data) {
    console.error(`Could not decode bus event`, value);
    return null;
  }

  return (
    <Card timestamp={timestamp} json={value}>
      {React.createElement(visualizer.Element, {
        ...props,
        value: [timestamp, data]
      })}
    </Card>
  );
};

export const EventVisualizer = {
  id: "Event",
  types: [BusEvent],
  options: StandardComponentOptions,
  Component: StandardComponent(EventElement),
  Element: EventElement
};
