import * as React from "react";
import { Canvas as CanvasR3F } from "react-three-fiber";
import { ContainerProps } from "react-three-fiber/targets/shared/web/ResizeContainer";
import styles from "./Canvas.module.scss";

export const Canvas: typeof CanvasR3F = (props: ContainerProps) => {
  return (
    <div className={styles.container}>
      <div className={styles.content}>
        <CanvasR3F
          style={{ background: "darkgray", width: "100%", height: "100%" }}
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
      </div>
    </div>
  );
};
