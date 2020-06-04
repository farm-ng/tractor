import { useState, useEffect } from "react";
import * as React from "react";
import { useLoader, ReactThreeFiber } from "react-three-fiber";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import { webSocketClient } from "../config";
import { Quaternion, Vector3 } from "three";
import { IWebSocketMessage } from "../models/IWebSocketMessage";
import { red, gray900 } from "./colors";
import { Status } from "../../genproto/farm_ng_proto/tractor/v1/status";

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
    webSocketClient.on("message", (message: IWebSocketMessage) => {
      const status = message as Status;
      if (!status.pose || !status.pose.position || !status.pose.rotation) {
        // eslint-disable-next-line no-console
        console.error("Unexpected status:", status);
        return;
      }
      const { x: tx, y: ty, z: tz } = status.pose.position;
      setPosition(new Vector3(tx, ty, tz));

      const { x: qx, y: qy, z: qz, w: qw } = status.pose.rotation;
      setQuaternion(new Quaternion(qx, qy, qz, qw));
    });
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
