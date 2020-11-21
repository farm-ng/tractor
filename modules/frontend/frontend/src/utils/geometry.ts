import { Matrix4 } from "three/src/math/Matrix4";
import {
  SE3Pose,
  NamedSE3Pose,
} from "@farm-ng/genproto-perception/farm_ng/perception/geometry";
import { matrix4ToSE3Pose, se3PoseToMatrix4 } from "./protoConversions";

export function rad2deg(radians: number): number {
  return (radians * 180) / Math.PI;
}

export function deg2rad(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

export function getInverse(pose: SE3Pose): SE3Pose {
  const original = se3PoseToMatrix4(pose);
  const inverse = new Matrix4();
  inverse.getInverse(original);
  return matrix4ToSE3Pose(inverse);
}

// Given a DAG consisting of a list of NamedSE3Poses, return a path (a list of node indices) from
// index 0 to a node matching the `targetFrameName`. Return null if no path is found.
// TODO: Implement this with BFS to get shortest path
function getDagPath(
  nodes: NamedSE3Pose[],
  targetFrameName: string
): number[] | null {
  const stack = [{ nodeIndex: 0, path: [0] }];
  const visited = nodes.map((_) => false);

  while (stack.length) {
    const { nodeIndex, path } = stack.pop() || {};

    if (nodeIndex === undefined || path === undefined) {
      return null;
    }

    if (visited[nodeIndex]) {
      continue;
    }

    if (nodes[nodeIndex].frameA === targetFrameName) {
      return path;
    }

    visited[nodeIndex] = true;

    const childrenIndices = nodes
      .map((n, i) => (n.frameA === nodes[nodeIndex].frameB ? i : undefined))
      .filter((_) => _ !== undefined) as number[];

    childrenIndices.forEach((childIndex) => {
      stack.push({ nodeIndex: childIndex, path: path.concat([childIndex]) });
    });
  }
  return null;
}

// Given a DAG consisting of a list of NamedSE3Poses, return the transform from the node at index 0 to
// a node with the `targetFrameName`. Return null if no path found.
// TODO: Write some tests
export function getDagTransform(
  inputNodes: NamedSE3Pose[],
  target: string
): SE3Pose | null {
  const nodes = [
    ...inputNodes,
    ...inputNodes.map((node) => ({
      frameA: node.frameB,
      frameB: node.frameA,
      aPoseB: getInverse(node.aPoseB as SE3Pose),
    })),
  ];
  const path = getDagPath(nodes, target);
  if (!path) {
    return null;
  }
  const transform = new Matrix4();
  while (path.length > 1) {
    const parentIndex = path.shift();
    if (parentIndex === undefined) {
      return null;
    }
    const parent = nodes[parentIndex];
    if (!parent?.aPoseB) {
      return null;
    }
    transform.multiply(se3PoseToMatrix4(parent.aPoseB));
  }
  return matrix4ToSE3Pose(transform);
}
