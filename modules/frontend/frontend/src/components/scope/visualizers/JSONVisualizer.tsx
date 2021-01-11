import * as React from "react";
import { ListGroup } from "react-bootstrap";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";

const JSONElement: React.FC<SingleElementVisualizerProps> = ({
  value: [timestamp, value],
}) => {
  return (
    <Card timestamp={timestamp}>
      <ListGroup horizontal>
        <ListGroup.Item>
          <pre>{JSON.stringify(value, null, 2)}</pre>
        </ListGroup.Item>
      </ListGroup>
    </Card>
  );
};

export const JSONVisualizer = {
  id: "JSON",
  types: "*" as const,
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(JSONElement),
  Element: JSONElement,
};
