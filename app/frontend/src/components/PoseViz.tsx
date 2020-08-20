import { useState, useEffect } from "react";
import * as React from "react";
import { Quaternion, Vector3 } from "three";
import { ipcClient } from "../config";

import {
  NamedSE3Pose,
  Vec3,
  Quaternion as Quat
} from "../../genproto/farm_ng_proto/tractor/v1/geometry";
import { Event } from "../../genproto/farm_ng_proto/tractor/v1/io";
import { Html } from "drei";

function ToVector3(v?: Vec3): Vector3 {
  if (!v) {
    return new Vector3(0, 0, 0);
  }
  return new Vector3(v.x, v.y, v.z);
}

function ToQuaternion(v?: Quat): Quaternion {
  if (!v) {
    return new Quaternion(0, 0, 0, 0);
  }
  return new Quaternion(v.x, v.y, v.z, v.w);
}

export const PoseViz: React.FC = () => {
  const [pose, setPose] = useState<NamedSE3Pose>(NamedSE3Pose.fromJSON({}));

  useEffect(() => {
    ipcClient.on(
      "type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose",
      (ev: Event) => {
        if (!ev.data) return;
        setPose(NamedSE3Pose.decode(ev.data.value));
      }
    );
  }, []);
  const position = ToVector3(pose.aPoseB?.position);
  const rotation = ToQuaternion(pose.aPoseB?.rotation);

  return (
    <group>
      <line>
        <geometry
          attach="geometry"
          vertices={[new Vector3(0, 0, 0), position]}
          onUpdate={(self) => (self.verticesNeedUpdate = true)}
        />
        <lineBasicMaterial attach="material" color="lightgray" />
      </line>
      <group position={position} quaternion={rotation}>
        <axesHelper>
          <Html scaleFactor={10}>
            <div>{pose.frameB}</div>
          </Html>
        </axesHelper>
      </group>
      <axesHelper>
        <Html scaleFactor={10}>
          <div>{pose.frameA}</div>
        </Html>
      </axesHelper>
    </group>
  );
};
