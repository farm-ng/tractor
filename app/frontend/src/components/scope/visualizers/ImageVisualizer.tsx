/* eslint-disable no-console */
import * as React from "react";
import { Card } from "react-bootstrap";
import {
  SingleElementVisualizerProps,
  Visualizer,
  VisualizerId,
  VisualizerOptionConfig,
  VisualizerProps
} from "../../../registry/visualization";
import { Image } from "../../../../genproto/farm_ng_proto/tractor/v1/image";
import { formatValue } from "../../../utils/formatValue";
import { EventTypeId } from "../../../registry/events";
import { useEffect, useState } from "react";
import { Layout } from "./Layout";
import { JsonPopover } from "../../JsonPopover";
import styles from "./ImageVisualizer.module.scss";

export const ImageElement: React.FC<SingleElementVisualizerProps<Image>> = ({
  value: [timestamp, value],
  resources
}) => {
  const [imgSrc, setImgSrc] = useState<string | null>(null);

  useEffect(() => {
    const fetchImage = async (): Promise<void> => {
      const resource = value.resource;
      if (resources && resource) {
        try {
          setImgSrc(await resources.getDataUrl(resource.path));
        } catch (e) {
          console.error(`Error loading resource ${resource.path}: ${e}`);
        }
      }
    };
    fetchImage();
  }, [value, resources]);

  return (
    <Card bg={"light"} className={[styles.card, "shadow-sm"].join(" ")}>
      <Card.Body>
        <img src={imgSrc || undefined} className={styles.image} />
      </Card.Body>
      <Card.Footer className={styles.footer}>
        <span className="text-muted">{formatValue(new Date(timestamp))}</span>
        <JsonPopover json={value} />
      </Card.Footer>
    </Card>
  );
};

export class ImageVisualizer implements Visualizer<Image> {
  static id: VisualizerId = "image";
  types: EventTypeId[] = ["type.googleapis.com/farm_ng_proto.tractor.v1.Image"];

  options: VisualizerOptionConfig[] = [
    { label: "view", options: ["overlay", "grid"] }
  ];

  component: React.FC<VisualizerProps<Image>> = (props) => {
    const view = props.options[0].value as "overlay" | "grid";
    return <Layout view={view} element={ImageElement} {...props} />;
  };
}
