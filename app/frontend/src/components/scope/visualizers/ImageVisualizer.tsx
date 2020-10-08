/* eslint-disable no-console */
import * as React from "react";
import {
  SingleElementVisualizerProps,
  Visualizer,
  VisualizerId,
  VisualizerOptionConfig,
  VisualizerProps
} from "../../../registry/visualization";
import { Image } from "../../../../genproto/farm_ng_proto/tractor/v1/image";
import { EventTypeId } from "../../../registry/events";
import { useEffect, useRef } from "react";
import { Layout } from "./Layout";
import styles from "./ImageVisualizer.module.scss";
import { Card } from "./Card";
import { useFetchDataUrl } from "../../../hooks/useFetchDataUrl";

export const ImageElement: React.FC<SingleElementVisualizerProps<Image>> = ({
  value: [timestamp, value],
  resources
}) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const isVideoFrame = value.resource?.contentType.startsWith("video");
  const mediaSrc = useFetchDataUrl(value.resource, resources || undefined);

  useEffect(() => {
    if (value && videoRef.current) {
      const currentTime = (value.frameNumber || 0) / (value.fps || 1);
      videoRef.current.currentTime = currentTime;
    }
  }, [value, videoRef]);

  return (
    <Card timestamp={timestamp} json={value}>
      {isVideoFrame ? (
        <video
          src={mediaSrc || undefined}
          ref={videoRef}
          className={styles.media}
        />
      ) : (
        <img src={mediaSrc || undefined} className={styles.media} />
      )}
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
