import * as React from "react";
import { Suspense, useState } from "react";
import { Canvas, PointerEvent } from "react-three-fiber";
import { Controls } from "./Controls";
import { Lights } from "./Lights";
import { Ground } from "./Ground";
import { Vector3 } from "three";
import { PoseViz } from "./PoseViz";

type Waypoints = Vector3[];

export const Scene: React.FC = () => {
  const [waypoints, setWaypoints] = useState<Waypoints>([]);

  const onGroundClick = (event: PointerEvent): void => {
    setWaypoints([...waypoints, event.point]);
  };

  return (
    <Canvas
      style={{ background: "darkgray", width: "100%", height: "100%" }}
      camera={{
        position: [2.5, 2.5, 2.5],
        fov: 60,
        near: 0.1,
        far: 500,
        up: [0, 0, 1]
      }}
    >
      <Lights />
      <Ground onClick={onGroundClick} />
      <fogExp2 args={[0xcccccc, 0.02]} />
      <Controls />
      <Suspense fallback={null}>
        <PoseViz />
      </Suspense>
    </Canvas>
  );
};
