/* eslint-disable no-console */
import * as React from "react";
import { Suspense } from "react";
import { useLoader } from "react-three-fiber";
import { NearestFilter, TextureLoader } from "three";
import {
  ApriltagRig,
  ApriltagRig_Node as ApriltagRigNode
} from "../../../../genproto/farm_ng_proto/tractor/v1/apriltag";
import { NamedSE3Pose } from "../../../../genproto/farm_ng_proto/tractor/v1/geometry";
import { SingleElementVisualizerProps } from "../../../registry/visualization";
import { getDagTransform } from "../../../utils/geometry";
import { NamedSE3PoseVisualizer } from "./NamedSE3Pose";
import {
  Standard3DComponent,
  Standard3DComponentOptions,
  Standard3DElement
} from "./StandardComponent";

const ApriltagRigNodeMarker3D: React.FC<{ value: ApriltagRigNode }> = ({
  value
}) => {
  const paddedId = String(value.id).padStart(5, "0");
  const textureUrl = `https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_${paddedId}.png`;
  const texture = useLoader(TextureLoader, textureUrl);
  texture.magFilter = NearestFilter;
  texture.minFilter = NearestFilter;

  // TODO(isherman): Use value.pointsTag, calculate UVs
  // const geometry = useMemo(() => {
  //   const quad = new Geometry();
  //   quad.vertices.push(toVector3(value.pointsTag[0]));
  //   quad.vertices.push(toVector3(value.pointsTag[1]));
  //   quad.vertices.push(toVector3(value.pointsTag[2]));
  //   quad.vertices.push(toVector3(value.pointsTag[3]));
  //   quad.faces.push(new Face3(0, 1, 2));
  //   quad.faces.push(new Face3(0, 2, 3));
  //   quad.computeFaceNormals();
  //   quad.computeVertexNormals();
  //   return quad;
  // }, [value]);
  return (
    <mesh>
      <planeBufferGeometry
        attach="geometry"
        args={[value.tagSize, value.tagSize]}
      />
      <meshBasicMaterial attach="material" map={texture} />
    </mesh>
  );
};

const ApriltagRig3DElement: React.FC<SingleElementVisualizerProps<
  ApriltagRig
>> = (props) => {
  const {
    value: [, value]
  } = props;

  const { nodes, rootTagId } = value;
  const rigRootName = nodes?.find((_) => _.id === rootTagId)?.frameName;
  const nodePoses = (nodes || [])
    .sort((a, b) => (a.id === rootTagId ? -1 : b.id === rootTagId ? 1 : 0))
    .map((_) => _.pose) as NamedSE3Pose[];

  const markers = nodes.map((node) => {
    const aPoseB = getDagTransform(nodePoses, node.frameName);
    if (!rigRootName || !aPoseB) {
      return undefined;
    }
    const pose: NamedSE3Pose = {
      frameA: rigRootName,
      frameB: node.frameName,
      aPoseB
    };

    return (
      <NamedSE3PoseVisualizer.Marker3D key={node.id} value={[0, pose]}>
        <Suspense fallback={null}>
          <ApriltagRigNodeMarker3D value={node} />
        </Suspense>
      </NamedSE3PoseVisualizer.Marker3D>
    );
  });
  return <group>{markers}</group>;
};

export const ApriltagRigVisualizer = {
  id: "ApriltagRig",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.ApriltagRig"],
  options: Standard3DComponentOptions,
  Component: Standard3DComponent(ApriltagRig3DElement),
  Element: Standard3DElement(ApriltagRig3DElement),
  Marker3D: ApriltagRig3DElement
};
