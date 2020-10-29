import * as React from "react";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { Image } from "../../../../genproto/farm_ng_proto/tractor/v1/image";
import { forwardRef, useEffect, useRef, useState } from "react";
import {
  StandardComponentOptions,
  StandardComponent
} from "./StandardComponent";
import { useFetchDataUrl } from "../../../hooks/useFetchDataUrl";
import styles from "./Image.module.scss";

// Provide a "bare" image for embedding in other components
// eslint-disable-next-line react/display-name
export const EmbeddableImage = forwardRef<
  HTMLDivElement,
  SingleElementVisualizerProps<Image>
>((props, ref) => {
  const {
    value: [, value],
    resources
  } = props;

  const videoRef = useRef<HTMLVideoElement>(null);
  const isVideoFrame = value.resource?.contentType.startsWith("video");
  const mediaSrc = useFetchDataUrl(value.resource, resources || undefined);
  const [videoError, setVideoError] = useState(false);
  const [imageError, setImageError] = useState(false);

  useEffect(() => {
    if (value && videoRef.current) {
      const currentTime = (value.frameNumber || 0) / (value.fps || 1);
      videoRef.current.currentTime = currentTime;
    }
  }, [value, videoRef]);

  return (
    <div ref={ref}>
      {imageError && <p>Could not load image.</p>}
      {videoError && (
        <p>Could not load image from video; may not be flushed to disk yet.</p>
      )}
      {isVideoFrame ? (
        <video
          src={mediaSrc || undefined}
          ref={videoRef}
          className={styles.media}
          onProgress={() => setVideoError(false)}
          onError={() => setVideoError(true)}
        />
      ) : (
        <img
          src={mediaSrc || undefined}
          className={styles.media}
          onLoad={() => setImageError(false)}
          onError={() => setImageError(true)}
        />
      )}
    </div>
  );
});

const ImageElement: React.FC<SingleElementVisualizerProps<Image>> = (props) => {
  const {
    value: [timestamp, value]
  } = props;
  return (
    <Card timestamp={timestamp} json={value}>
      <EmbeddableImage {...props} />
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
