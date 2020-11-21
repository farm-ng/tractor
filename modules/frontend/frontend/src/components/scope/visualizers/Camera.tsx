import * as React from "react";
import {
  CameraHelper,
  PerspectiveCamera as PerspectiveCameraImpl,
  Quaternion,
  Vector3
} from "three";
import { forwardRef, useLayoutEffect } from "react";
import { useThree, useUpdate } from "react-three-fiber";
import mergeRefs from "react-merge-refs";
import { useHelper } from "drei";

type Props = JSX.IntrinsicElements["perspectiveCamera"] & {
  makeDefault?: boolean;
  showHelper?: boolean;
};

export const PerspectiveCamera = forwardRef(
  ({ makeDefault = false, showHelper = false, ...props }: Props, ref) => {
    const { setDefaultCamera, camera, size } = useThree();
    const cameraRef = useUpdate<PerspectiveCameraImpl>(
      (cam) => {
        if (makeDefault) {
          cam.aspect = size.width / size.height;
          cam.updateProjectionMatrix();
        }
      },
      [size, props]
    );

    if (showHelper) {
      useHelper(cameraRef, CameraHelper);
    }

    useLayoutEffect(() => {
      if (makeDefault && cameraRef.current) {
        const oldCam = camera;
        setDefaultCamera(cameraRef.current);
        // `setDefaultCamera` sets `lookAt` to the origin. We choose to look down the provided z-axis instead.
        const { quaternion } = props;
        if (quaternion) {
          // TODO: Why is this not {0, 0, 1}?
          const lookAtPoint = new Vector3(1, 0, 0);
          lookAtPoint.applyQuaternion(
            quaternion instanceof Quaternion
              ? quaternion
              : new Quaternion(...quaternion)
          );
          cameraRef.current.lookAt(lookAtPoint);
          cameraRef.current.updateProjectionMatrix();
        }

        return () => setDefaultCamera(oldCam);
      }
    }, [camera, cameraRef, makeDefault, setDefaultCamera]);

    return (
      <group>
        <perspectiveCamera ref={mergeRefs([cameraRef, ref])} {...props} />
      </group>
    );
  }
);
PerspectiveCamera.displayName = "PerspectiveCamera";
