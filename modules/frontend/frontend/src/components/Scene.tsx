import * as React from "react";
import { Suspense, useState } from "react";
import { PointerEvent } from "react-three-fiber";
import { Controls } from "./Controls";
import { Lights } from "./Lights";
import { Ground } from "./Ground";
import { Vector3 } from "three";
import { PoseViz } from "./PoseViz";
import { Canvas } from "./Canvas";

type Waypoints = Vector3[];

export const Scene: React.FC = () => {
  const [waypoints, setWaypoints] = useState<Waypoints>([]);

  const onGroundClick = (event: PointerEvent): void => {
    setWaypoints([...waypoints, event.point]);
  };

  return (
    <Canvas>
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
