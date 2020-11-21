import * as React from "react";

export const Lights: React.FC = () => {
  return (
    <group>
      <directionalLight color="white" position={[1, 1, 1]} />
      <directionalLight color="lightblue" position={[-1, -1, -1]} />
      <ambientLight color="darkgray" />
    </group>
  );
};
