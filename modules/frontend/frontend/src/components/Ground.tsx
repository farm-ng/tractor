import * as React from "react";
import { PointerEvent } from "react-three-fiber";

type GroundProps = {
  onClick?: (event: PointerEvent) => void;
  transparent?: boolean;
};

export const Ground: React.FC<GroundProps> = ({ transparent }) => {
  const groundPlane = (
    <mesh receiveShadow>
      <planeBufferGeometry attach="geometry" args={[1000, 1000]} />
      <meshPhongMaterial attach="material" color={"gray"} />
    </mesh>
  );

  return (
    <group position={[0, 0, -0.002]}>
      <gridHelper
        args={[400, 400]}
        position={[0, 0, 0.001]}
        rotation={[Math.PI / 2, 0, 0]}
      />
      {!transparent && groundPlane}
    </group>
  );
};
