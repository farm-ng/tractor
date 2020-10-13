import * as React from "react";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { Image } from "../../../../genproto/farm_ng_proto/tractor/v1/image";
import { useEffect, useRef } from "react";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import styles from "./Image.module.scss";
import { useFetchDataUrl } from "../../../hooks/useFetchDataUrl";

const ImageElement: React.FC<SingleElementVisualizerProps<Image>> = ({
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

export const ImageVisualizer = {
  id: "Image",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.Image"],
  options: StandardComponentOptions,
  Component: StandardComponent(ImageElement),
  Element: ImageElement
};
