import { useState, useEffect } from "react";
import * as React from "react";
import { useLoader, ReactThreeFiber } from "react-three-fiber";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import { red, gray900 } from "./colors";
import { Quaternion, Vector3 } from "three";

export const Tractor: React.FC = () => {
  // TODO: Should this be bundled?
  const stl = useLoader(STLLoader, "./stl/tractor.v0.stl");

  const [position, setPosition] = useState<ReactThreeFiber.Vector3>(
    new Vector3()
  );
  const [quaternion, setQuaternion] = useState<THREE.Quaternion>(
    new Quaternion()
  );

  useEffect(() => {
    setPosition(new Vector3(0, 0, 0));
    setQuaternion(new Quaternion(0, 0, 0, 0));
  }, []);

  return (
    <group position={position} quaternion={quaternion}>
      <axesHelper />
      <mesh castShadow receiveShadow scale={[0.001, 0.001, 0.001]}>
        <bufferGeometry attach="geometry" {...stl} />
        <meshPhongMaterial
          attach="material"
          color={red}
          specular={gray900}
          shininess={200}
        />
      </mesh>
    </group>
  );
};
