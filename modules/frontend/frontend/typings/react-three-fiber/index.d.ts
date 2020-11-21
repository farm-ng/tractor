/* eslint-disable @typescript-eslint/no-unused-vars */
import { ReactThreeFiber } from "react-three-fiber";
import { FogExp2 } from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { EffectComposer } from "three/examples/jsm/postprocessing/EffectComposer";
import { GlitchPass } from "three/examples/jsm/postprocessing/GlitchPass";
import { RenderPass } from "three/examples/jsm/postprocessing/RenderPass";
import { ShaderPass } from "three/examples/jsm/postprocessing/ShaderPass";
import { UnrealBloomPass } from "three/examples/jsm/postprocessing/UnrealBloomPass";
import { SSAOPass } from "three/examples/jsm/postprocessing/SSAOPass";

declare global {
  namespace JSX {
    interface IntrinsicElements {
      orbitControls: ReactThreeFiber.Object3DNode<
        OrbitControls,
        typeof OrbitControls
      >;
      fogExp2: ReactThreeFiber.Node<FogExp2, typeof FogExp2>;
      effectComposer: ReactThreeFiber.Node<
        EffectComposer,
        typeof EffectComposer
      >;
      renderPass: ReactThreeFiber.Node<RenderPass, typeof RenderPass>;
      glitchPass: ReactThreeFiber.Node<GlitchPass, typeof GlitchPass>;
      shaderPass: ReactThreeFiber.Node<ShaderPass, typeof ShaderPass>;
      unrealBloomPass: ReactThreeFiber.Node<
        UnrealBloomPass,
        typeof UnrealBloomPass
      >;
      sSAOPass: ReactThreeFiber.Node<SSAOPass, typeof SSAOPass>;
    }
  }
}
