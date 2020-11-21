import * as React from "react";
import { useEffect, useRef } from "react";
import { extend, useFrame, useThree } from "react-three-fiber";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { ShaderPass } from "three/examples/jsm/postprocessing/ShaderPass";
import { VignetteShader } from "three/examples/jsm/shaders/VignetteShader";
extend({ EffectComposer, RenderPass, ShaderPass });

export const FisheyeEffect: React.FC = () => {
  const { gl, scene, camera, size } = useThree();
  const composer = useRef<EffectComposer>();
  useEffect(() => {
    if (composer.current) {
      composer.current.setSize(size.width, size.height);
    }
  }, [size]);

  useFrame(() => {
    if (composer.current) {
      composer.current.render();
    }
  }, 1);

  return (
    <effectComposer ref={composer} args={[gl]}>
      <renderPass attachArray="passes" scene={scene} camera={camera} />
      <shaderPass
        attachArray="passes"
        args={[VignetteShader]}
        uniforms-darkness-value={1}
      />
    </effectComposer>
  );
};
