import * as React from "react";
import { useEffect, useRef } from "react";
import { useObserver } from "mobx-react-lite";

import { ApriltagDetections } from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import { autorun } from "mobx";
import styles from "./Video.module.scss";
import { useStores } from "../hooks/useStores";
import { drawAprilTagDetections } from "../utils/drawApriltagDetections";

const t265Resolution = {
  width: 848,
  height: 800,
};

export const Video: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const videoRef = useRef<HTMLVideoElement>(null);
  const { busEventStore, mediaStreamStore } = useStores();

  const resize = (): void => {
    const videoElement = videoRef?.current;
    const canvasElement = canvasRef?.current;
    if (videoElement && canvasElement) {
      canvasElement.width = videoElement.clientWidth;
      canvasElement.height = videoElement.clientHeight;
    }
  };

  useEffect(
    () =>
      autorun(() => {
        const videoElement = videoRef?.current;
        if (!videoElement) {
          return;
        }
        videoElement.srcObject = mediaStreamStore.videoStream;
      }),
    [videoRef]
  );

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
        drawAprilTagDetections(
          busEventStore.lastSnapshot.get("tracking_camera/front/apriltags")
            ?.latestEvent as ApriltagDetections,
          ctx,
          canvas,
          t265Resolution
        );
      }),
    [canvasRef]
  );

  return useObserver(() => (
    <div className={styles.content}>
      <div className={styles.annotatedVideoContainer}>
        <div className={styles.annotatedVideo}>
          <video ref={videoRef} autoPlay muted className={styles.video}></video>
          <canvas ref={canvasRef} className={styles.canvas}></canvas>
        </div>
      </div>
    </div>
  ));
};
