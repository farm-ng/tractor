from typing import Tuple

from liegroups import SE3

from farm_ng_proto.tractor.v1.tractor_pb2 import TractorConfig


class TractorKinematics:
    def __init__(self, tractor_config: TractorConfig):
        self.wheel_radius = tractor_config.wheel_radius.value
        self.wheel_base_line = tractor_config.wheel_baseline.value

    def wheel_velocity_to_unicycle(self, v_left: float, v_right: float) -> Tuple[float, float]:
        v = (self.wheel_radius/2.0)*(v_left+v_right)
        w = (self.wheel_radius/self.wheel_base_line)*(v_right-v_left)
        return v, w

    def unicycle_to_wheel_velocity(self, v: float, w: float) -> Tuple[float, float]:
        v_right = (2*v + w*self.wheel_base_line)/(2*self.wheel_radius)
        v_left = (2*v - w*self.wheel_base_line)/(2*self.wheel_radius)
        return v_left, v_right

    def compute_tractor_pose_delta(self, v_left: float, v_right: float, delta_t: float) -> SE3:
        v, w = self.wheel_velocity_to_unicycle(v_left, v_right)
        tractor_pose_delta = SE3.exp([v*delta_t, 0, 0, 0, 0, w*delta_t])
        return tractor_pose_delta

    def evolve_world_pose_tractor(
            self, world_pose_tractor: SE3, v_left: float,
            v_right: float, delta_t: float,
    ) -> SE3:
        tractor_pose_delta = self.compute_tractor_pose_delta(v_left, v_right, delta_t)
        return world_pose_tractor.dot(tractor_pose_delta)
