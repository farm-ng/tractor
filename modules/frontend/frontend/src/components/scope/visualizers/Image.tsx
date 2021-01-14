import * as React from "react";
import { Card } from "./Card";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { Image } from "@farm-ng/genproto-perception/farm_ng/perception/image";
import { forwardRef, useEffect, useRef, useState } from "react";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { useFetchDataUrl } from "../../../hooks/useFetchDataUrl";
import styles from "./Image.module.scss";

interface ImageProps extends SingleElementVisualizerProps<Image> {
  depth?: boolean;
}

// Provide a "bare" image for embedding in other components
// eslint-disable-next-line react/display-name
export const EmbeddableImage = forwardRef<HTMLDivElement, ImageProps>(
  (props, ref) => {
    const {
      value: [, value],
      depth,
      resources,
    } = props;

    const resource = depth ? value.depthmap?.resource : value.resource;

    const videoRef = useRef<HTMLVideoElement>(null);
    const isVideoFrame = resource?.contentType.startsWith("video");
    const mediaSrc = useFetchDataUrl(resource, resources || undefined);
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
          <p>
            Could not load image from video; may not be flushed to disk yet.
          </p>
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
  }
);

const ImageElement: React.FC<ImageProps> = (props) => {
  const {
    value: [timestamp, value],
  } = props;
  return (
    <Card timestamp={timestamp} json={value}>
      <EmbeddableImage {...props} />
    </Card>
  );
};

export const ImageVisualizer = {
  id: "Image",
  types: ["type.googleapis.com/farm_ng.perception.Image"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(ImageElement),
  Element: ImageElement,
};
