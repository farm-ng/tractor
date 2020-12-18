/* eslint-disable no-console */
import * as React from "react";
import { useState, useEffect, useRef } from "react";
import { Card } from "./Card";
import styles from "./ApriltagDetections.module.scss";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { ApriltagDetections } from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import { autorun } from "mobx";
import { drawAprilTagDetections } from "../../../utils/drawApriltagDetections";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";
import { EmbeddableImage } from "./Image";

const ApriltagDetectionsElement: React.FC<SingleElementVisualizerProps<
  ApriltagDetections
>> = ({ value: [timestamp, value], resources }) => {
  const [imgSrc, setImgSrc] = useState<string>();
  const imageRef = useRef<HTMLImageElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const fetchImage = async (): Promise<void> => {
      const resource = value.image?.resource;
      if (resources && resource && resource.path) {
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
    <Card json={value} timestamp={timestamp}>
      <div className={styles.annotatedImageContainer}>
        <div className={styles.annotatedImage}>
          {value.image && (
            <EmbeddableImage
              ref={imageRef}
              resources={resources}
              value={[timestamp, value.image]}
            />
          )}
          <canvas ref={canvasRef} className={styles.canvas}></canvas>
        </div>
      </div>
    </Card>
  );
};

export const ApriltagDetectionsVisualizer = {
  id: "ApriltagDetections",
  types: ["type.googleapis.com/farm_ng.perception.ApriltagDetections"],
  options: StandardComponentOptions,
  Component: StandardComponent(ApriltagDetectionsElement),
  Element: ApriltagDetectionsElement,
};
