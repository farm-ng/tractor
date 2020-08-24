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
import { useFrame } from "react-three-fiber";

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

type ReferenceFrameNode = {
  pose: NamedSE3Pose;
  children: ReferenceFrameNode[];
  parent?: ReferenceFrameNode;
};

const findFrameB = function (
  root: ReferenceFrameNode,
  frameB: string
): ReferenceFrameNode | undefined {
  if (root.pose.frameB === frameB) {
    return root;
  } else {
    for (const x of root.children) {
      const y = findFrameB(x, frameB);
      if (y) {
        return y;
      }
    }
  }
  return undefined;
};

export const ReferenceFrameViz: React.FC<ReferenceFrameNode> = (state) => {
  const position = ToVector3(state.pose.aPoseB?.position);
  const rotation = ToQuaternion(state.pose.aPoseB?.rotation);
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
            <div>{state.pose.frameB}</div>
          </Html>
        </axesHelper>
        {state.children.map((x: ReferenceFrameNode, item: number) => (
          <ReferenceFrameViz {...x} key={item} />
        ))}
      </group>
    </group>
  );
};

export const PoseViz: React.FC = () => {
  const [root, setRootNode] = useState<ReferenceFrameNode>({
    pose: NamedSE3Pose.fromJSON({ frameA: "root", frameB: "odometry/wheel" }),
    children: [],
    parent: undefined
  });
  useFrame((_state) => {
    setRootNode({ ...root });
  });
  useEffect(() => {
    ipcClient.on(
      "type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose",
      (ev: Event) => {
        if (!ev.data) return;
        const pose = NamedSE3Pose.decode(ev.data.value);
        const pose_node = findFrameB(root, pose.frameB);
        if (pose_node && pose_node.pose.frameA == pose.frameA) {
          pose_node.pose = pose;
        } else {
          const parent = findFrameB(root, pose.frameA);
          if (!parent) return;
          parent.children.push({ pose: pose, parent: parent, children: [] });
        }
      }
    );
  }, []);

  return <ReferenceFrameViz {...root} />;
};
