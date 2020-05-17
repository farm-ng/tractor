from typing import Tuple

import numpy as np
from liegroups import SE3

from .kinematics import TractorKinematics


class TractorMoveToGoalController:
    def __init__(self, tractor_model: TractorKinematics):
        self.tractor_model = tractor_model
        self.K_w = 20
        self.K_v = 1.0
        self.new_goal = True

    def set_goal(self, world_pose_tractor: SE3):
        self.world_pose_tractor_goal = world_pose_tractor
        self.new_goal = True

    def update(self, world_pose_tractor_est) -> Tuple[float, float]:
        tractor_pose_goal = world_pose_tractor_est.inv().dot(
            self.world_pose_tractor_goal,
        )
        distance = np.linalg.norm(tractor_pose_goal.trans)
        if distance < 0.05:
            return 0, 0

        trans_g = tractor_pose_goal.trans
        heading_error = np.arctan2(trans_g[1], trans_g[0])
        w = np.clip(self.K_w * heading_error, -np.pi/4, np.pi/4)
        v = np.clip(self.K_v*distance, 0.0, 1.0)
        if self.new_goal:
            if np.abs(heading_error) > np.pi/64:
                v = 0  # turn in place to goal
            else:
                self.new_goal = False
        return self.tractor_model.unicycle_to_wheel_velocity(v, w)
