import * as React from "react";
import { PointerEvent } from "react-three-fiber";

type GroundProps = {
  onClick: (event: PointerEvent) => void;
};

export const Ground: React.FC<GroundProps> = (props) => {
  const onClick = (event: PointerEvent): void => {
     if (!event.ctrlKey) {
           return;
     }
    props.onClick(event);
  };

  return (
    <group>
      <axesHelper />
      <gridHelper
        args={[400, 400]}
        position={[0, 0, 0.1]}
        rotation={[Math.PI / 2, 0, 0]}
      />
      <mesh receiveShadow onClick={onClick}>
        <planeBufferGeometry attach="geometry" args={[1000, 1000]} />
        <meshPhongMaterial attach="material" color="#dddddd" />
      </mesh>
    </group>
  );
};
