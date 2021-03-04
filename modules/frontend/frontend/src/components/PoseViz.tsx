import { useState, useEffect } from "react";
import * as React from "react";
import { Vector3 } from "three";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { NamedSE3Pose } from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import { Html } from "drei";
import { useFrame } from "react-three-fiber";
import { decodeAnyEvent } from "../models/decodeAnyEvent";
import { useStores } from "../hooks/useStores";
import { toQuaternion, toVector3 } from "../utils/protoConversions";

type ReferenceFrameNode = {
  pose: NamedSE3Pose;
  children: ReferenceFrameNode[];
  parent?: ReferenceFrameNode;
  textScale?: number;
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

export const ReferenceFrameViz: React.FC<ReferenceFrameNode> = (props) => {
  const position = toVector3(props.pose.aPoseB?.position);
  const rotation = toQuaternion(props.pose.aPoseB?.rotation);
  return (
    <group>
      <line>
        <geometry
          attach="geometry"
          vertices={[new Vector3(0, 0, 0), position]}
          onUpdate={(self: any) => (self.verticesNeedUpdate = true)}
        />
        <lineBasicMaterial attach="material" color="lightgray" />
      </line>
      <group position={position} quaternion={rotation}>
        <axesHelper>
          <Html scaleFactor={props.textScale || 1}>
            <div>{props.pose.frameB}</div>
          </Html>
        </axesHelper>
        {props.children.map((x: ReferenceFrameNode, item: number) => (
          <ReferenceFrameViz {...x} textScale={10} key={item} />
        ))}
      </group>
    </group>
  );
};

export const PoseViz: React.FC = () => {
  const [root, setRootNode] = useState<ReferenceFrameNode>({
    pose: NamedSE3Pose.fromJSON({ frameA: "root", frameB: "odometry/wheel" }),
    children: [],
    parent: undefined,
  });
  useFrame((_state) => {
    setRootNode({ ...root });
  });

  const { busEventStore } = useStores();
  const busEventEmitter = busEventStore.transport;

  useEffect(() => {
    const handle = busEventEmitter.on(
      "type.googleapis.com/farm_ng.perception.NamedSE3Pose",
      (event: BusEvent) => {
        const pose = decodeAnyEvent(event) as NamedSE3Pose;

        if (!pose) return;
        const poseNode = findFrameB(root, pose.frameB);
        if (poseNode && poseNode.pose.frameA == pose.frameA) {
          poseNode.pose = pose;
        } else {
          const parent = findFrameB(root, pose.frameA);
          if (!parent) return;
          parent.children.push({ pose: pose, parent: parent, children: [] });
        }
      }
    );
    return () => {
      handle.unsubscribe();
    };
  }, [busEventEmitter]);

  return <ReferenceFrameViz {...root} textScale={10} />;
};
