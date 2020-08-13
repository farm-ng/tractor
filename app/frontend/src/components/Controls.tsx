import * as React from "react";
import { useRef } from "react";
import { extend, useThree, useFrame } from "react-three-fiber";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";

extend({ OrbitControls });

export const Controls: React.FC = () => {
  const ref = useRef<OrbitControls>();
  const { camera, gl } = useThree();

  useFrame(() => ref.current && ref.current.update());

  return (
    <orbitControls
      ref={ref}
      args={[camera, gl.domElement]}
      enableDamping
      dampingFactor={0.05}
      screenSpacePanning={false}
      maxDistance={100}
      minDistance={0.1}
      maxPolarAngle={Math.PI / 2}
    />
  );
};
