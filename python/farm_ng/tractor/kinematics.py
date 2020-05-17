from typing import Tuple

import numpy as np
from liegroups import SE3


class TractorKinematics:
    def __init__(self):
        self.wheel_radius = 0.0254*15/2.0
        self.wheel_base_line = 0.0254*46

    def wheel_velocity_to_unicycle(self, v_left: float, v_right: float) -> Tuple[float, float]:
        v = (self.wheel_radius/2.0)*(v_left+v_right)
        w = (self.wheel_radius/self.wheel_base_line)*(v_right-v_left)
        return v, w

    def unicycle_to_wheel_velocity(self, v: float, w: float) -> Tuple[float, float]:
        v_right = (2*v + w*self.wheel_base_line)/(2*self.wheel_radius)
        v_left = (2*v - w*self.wheel_base_line)/(2*self.wheel_radius)
        return v_left, v_right

    def evolve_world_pose_tractor(
        self, world_pose_tractor: SE3, v_left: float,
        v_right: float, delta_t: float,
    ) -> SE3:
        v, w = self.wheel_velocity_to_unicycle(v_left, v_right)
        tractor_pose_t = SE3.exp([v*delta_t, 0, 0, 0, 0, w*delta_t])
        return world_pose_tractor.dot(tractor_pose_t)
