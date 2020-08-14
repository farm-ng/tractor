import * as React from "react";
import { Suspense, useState } from "react";
import { Canvas, PointerEvent } from "react-three-fiber";
import { Tractor } from "./Tractor";
import { Controls } from "./Controls";
import { Lights } from "./Lights";
import { Ground } from "./Ground";
import { Vector3 } from "three";
import { gray300 } from "./colors";

type Waypoints = Vector3[];

export const Scene: React.FC = () => {
  const [waypoints, setWaypoints] = useState<Waypoints>([]);

  const onGroundClick = (event: PointerEvent): void => {
    setWaypoints([...waypoints, event.point]);
  };

  return (
    <Canvas
      style={{ background: gray300, height: "400px", width: "400px" }}
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
      <fogExp2 args={[gray300.getHex(), 0.02]} />
      <Controls />
      <Suspense fallback={null}>
        <Tractor />
      </Suspense>
    </Canvas>
  );
};
