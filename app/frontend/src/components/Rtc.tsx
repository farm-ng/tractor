/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable no-console */
import * as React from "react";
import { useEffect, useState, useRef } from "react";

import { Vec2 } from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import { ApriltagDetections } from "../../genproto/farm_ng_proto/tractor/v1/apriltag";
import { useWebRTC } from "../hooks/useWebRTC";
import { BusEvent } from "../models/BusEvent";
import { decodeAnyEvent } from "../models/decodeAnyEvent";

type TractorState = {
  [key: string]: BusEvent;
};

const t265Resolution = {
  width: 848,
  height: 800
};

const drawAprilTagDetections = (
  detections: ApriltagDetections | null,
  ctx: CanvasRenderingContext2D,
  canvas: HTMLCanvasElement
): void => {
  ctx.fillStyle = "#ff0000";
  ctx.strokeStyle = "#ff0000";
  ctx.lineWidth = 4;
  ctx.font = "16px Arial";
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const toCanvasCoords = (v: Vec2): Vec2 => ({
    x: v.x * (canvas.width / t265Resolution.width),
    y: v.y * (canvas.height / t265Resolution.height)
  });
  if (detections) {
    detections.detections.forEach((d) => {
      const pCoords = d.p.map(toCanvasCoords);
      ctx.beginPath();
      ctx.moveTo(pCoords[0].x, pCoords[0].y);
      ctx.lineTo(pCoords[1].x, pCoords[1].y);
      ctx.lineTo(pCoords[2].x, pCoords[2].y);
      ctx.lineTo(pCoords[3].x, pCoords[3].y);
      ctx.closePath();
      ctx.stroke();
      if (d.c) {
        const cCoords = toCanvasCoords(d.c);
        ctx.fillText(d.id.toString(), cCoords.x, cCoords.y);
      }
    });
  }
};

export const Rtc: React.FC = () => {
  const [tractorState, setTractorState] = useState<TractorState>({});
  const [
    apriltagDetections,
    setApriltagDetections
  ] = useState<ApriltagDetections | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const videoRef = useRef<HTMLVideoElement | null>(null);

  const resize = (): void => {
    const videoElement = videoRef?.current;
    const canvasElement = canvasRef?.current;
    if (videoElement && canvasElement) {
      canvasElement.width = videoElement.clientWidth;
      canvasElement.height = videoElement.clientHeight;
    }
  };

  const startVideo = (): void => {
    const videoElement = videoRef?.current;
    if (videoElement) {
      videoElement.play();
    }
  };

  const busEventEmitter = useWebRTC(
    `http://${window.location.hostname}:9900/twirp/farm_ng_proto.tractor.v1.WebRTCProxyService/InitiatePeerConnection`,
    videoRef
  );

  busEventEmitter.on("*", (event) => {
    const value = decodeAnyEvent(event);
    if (value) {
      setTractorState((state) => ({
        ...state,
        [event.name]: value
      }));
    }
    if (
      event.data?.typeUrl ===
      "type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagDetections"
    ) {
      setApriltagDetections(value as ApriltagDetections);
    }
  });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }

    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    resize();
    drawAprilTagDetections(apriltagDetections, ctx, canvas);
  }, [canvasRef, apriltagDetections]);

  return (
    <div style={{ display: "flex", flexDirection: "row" }}>
      <div id="annotatedVideo" style={{ position: "relative", width: "50%" }}>
        <div id="videos">
          <video ref={videoRef} autoPlay style={{ width: "100%" }}></video>
        </div>
        <canvas
          ref={canvasRef}
          style={{ position: "absolute", top: 0, left: 0, bottom: 0, right: 0 }}
        ></canvas>
      </div>
      <div style={{ display: "flex", flexDirection: "column" }}>
        <button onClick={startVideo}>Start Video</button>
        <button disabled>Send Test Event</button>
      </div>
      <div style={{ color: "white", width: "50%" }}>
        {Object.keys(tractorState).map((key, i) => (
          <React.Fragment key={i}>
            <span>Key Name: {key} </span>
            <p>
              {Object.keys(tractorState[key]).map((keyJ, _j) => (
                <React.Fragment key={keyJ}>
                  <span> {keyJ} </span>
                  <span>
                    {JSON.stringify((tractorState[key] as any)[keyJ])}{" "}
                  </span>
                </React.Fragment>
              ))}
            </p>
          </React.Fragment>
        ))}
      </div>
    </div>
  );
};
