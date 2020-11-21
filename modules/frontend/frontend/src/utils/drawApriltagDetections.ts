import { ApriltagDetections } from "@farm-ng/genproto-perception/farm_ng/perception/apriltag";
import { Vec2 } from "@farm-ng/genproto-perception/farm_ng/perception/geometry";

export const drawAprilTagDetections = (
  detections: ApriltagDetections | null,
  ctx: CanvasRenderingContext2D,
  canvas: HTMLCanvasElement,
  scaleFactor?: { width: number; height: number },
  lineWidth?: number
): void => {
  ctx.fillStyle = "#ff0000";
  ctx.strokeStyle = "#ff0000";
  ctx.lineWidth = lineWidth || 4;
  ctx.font = "16px Arial";
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const toCanvasCoords = (v: Vec2): Vec2 => ({
    x: v.x * (scaleFactor ? canvas.width / scaleFactor.width : 1),
    y: v.y * (scaleFactor ? canvas.height / scaleFactor.height : 1),
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
