import * as React from "react";
import { white, blue900, gray800 } from "./colors";

export const Lights: React.FC = () => {
  return (
    <group>
      <directionalLight color={white} position={[1, 1, 1]} />
      <directionalLight color={blue900} position={[-1, -1, -1]} />
      <ambientLight color={gray800} />
    </group>
  );
};
