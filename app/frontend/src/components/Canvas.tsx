import * as React from "react";
import { Canvas as CanvasR3F } from "react-three-fiber";
import { ContainerProps } from "react-three-fiber/targets/shared/web/ResizeContainer";

export const Canvas: typeof CanvasR3F = (props: ContainerProps) => {
  return (
    <CanvasR3F
      style={{ background: "darkgray", width: "100%", height: "auto" }}
      camera={{
        position: [2.5, 2.5, 2.5],
        fov: 60,
        near: 0.1,
        far: 500,
        up: [0, 0, 1]
      }}
      {...props}
    >
      {props.children}
    </CanvasR3F>
  );
};
