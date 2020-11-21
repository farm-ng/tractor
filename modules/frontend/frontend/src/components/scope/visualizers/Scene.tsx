import * as React from "react";
import { Canvas } from "../../Canvas";
import { Controls } from "../../Controls";
import { Ground } from "../../Ground";
import { Lights } from "../../Lights";

interface IProps {
  controls?: boolean;
  ground?: boolean;
  groundTransparency?: boolean;
}

export const Scene: React.FC<IProps> = ({
  controls = true,
  ground = true,
  groundTransparency = false,
  children
}) => {
  return (
    <Canvas>
      <Lights />
      {ground && <Ground transparent={groundTransparency} />}
      <fogExp2 args={[0xcccccc, 0.02]} />
      {controls && <Controls />}
      {children}
    </Canvas>
  );
};
