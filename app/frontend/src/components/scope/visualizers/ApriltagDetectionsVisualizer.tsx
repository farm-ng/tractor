/* eslint-disable no-console */
import * as React from "react";
import { useState, useEffect, useRef } from "react";
import { Card } from "react-bootstrap";
import styles from "./ApriltagDetectionsVisualizer.module.scss";
import {
  SingleElementVisualizerProps,
  Visualizer,
  VisualizerId,
  VisualizerOptionConfig,
  VisualizerProps
} from "../../../registry/visualization";
import { formatValue } from "../../../utils/formatValue";
import { ApriltagDetections } from "../../../../genproto/farm_ng_proto/tractor/v1/apriltag";
import { EventTypeId } from "../../../registry/events";
import { autorun } from "mobx";
import { drawAprilTagDetections } from "../../../utils/drawApriltagDetections";
import { JsonPopover } from "../../JsonPopover";
import { Layout } from "./Layout";

export const ApriltagDetectionsElement: React.FC<SingleElementVisualizerProps<
  ApriltagDetections
>> = ({ value: [timestamp, value], resources }) => {
  const [imgSrc, setImgSrc] = useState<string | null>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const fetchImage = async (): Promise<void> => {
      const resource = value.image?.resource;
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

  const resize = (): void => {
    const imageElement = imageRef?.current;
    const canvasElement = canvasRef?.current;
    if (imageElement && canvasElement) {
      canvasElement.width = imageElement.clientWidth;
      canvasElement.height = imageElement.clientHeight;
    }
  };

  useEffect(
    () =>
      autorun(() => {
        const canvas = canvasRef.current;
        if (!canvas) {
          return;
        }

        const ctx = canvas.getContext("2d");
        if (!ctx) {
          return;
        }

        resize();

        const { imageWidth: width, imageHeight: height } =
          value.image?.cameraModel || {};

        drawAprilTagDetections(
          value,
          ctx,
          canvas,
          width && height ? { width, height } : undefined,
          2
        );
      }),
    [canvasRef, value, imgSrc]
  );

  return (
    <Card bg={"light"} className={[styles.card, "shadow-sm"].join(" ")}>
      <Card.Body>
        <div className={styles.annotatedImageContainer}>
          <div className={styles.annotatedImage}>
            <img
              ref={imageRef}
              src={imgSrc || undefined}
              className={styles.image}
            />
            <canvas ref={canvasRef} className={styles.canvas}></canvas>
          </div>
        </div>
      </Card.Body>
      <Card.Footer className={styles.footer}>
        <span className="text-muted">{formatValue(new Date(timestamp))}</span>
        <JsonPopover json={value} />
      </Card.Footer>
    </Card>
  );
};

export class ApriltagDetectionsVisualizer
  implements Visualizer<ApriltagDetections> {
  static id: VisualizerId = "apriltagDetections";
  types: EventTypeId[] = [
    "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagDetections"
  ];

  options: VisualizerOptionConfig[] = [
    { label: "view", options: ["overlay", "grid"] }
  ];

  component: React.FC<VisualizerProps<ApriltagDetections>> = (props) => {
    const view = props.options[0].value as "overlay" | "grid";
    return (
      <Layout view={view} element={ApriltagDetectionsElement} {...props} />
    );
  };
}
