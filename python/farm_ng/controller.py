from typing import Tuple

import numpy as np
from liegroups import SE3


class TractorMoveToGoalController:
    def __init__(self):
        self.K_w = 5
        self.K_v = 0.25

        self.v = 0.0
        self.v_acc = 2.0/50

        self.w = 0.0
        self.w_acc = (np.pi/2)/50
        self.world_pose_tractor_goal = None
        self.max_v = 0.5
        self.max_w = np.pi/16

    def reset(self):
        self.world_pose_tractor_goal = None
        self.v = 0
        self.w = 0

    def set_goal(self, world_pose_tractor: SE3):
        self.world_pose_tractor_goal = world_pose_tractor

    def update_vw(self, v, w):
        self.w += np.clip(w - self.w, -self.w_acc, self.w_acc)
        self.v += np.clip(v - self.v, -self.v_acc, self.v_acc)
        self.w = np.clip(self.w, -self.max_w, self.max_w)
        self.v = np.clip(self.v, -self.max_v, self.max_v)
        return self.v, self.w

    def update(self, world_pose_tractor_est) -> Tuple[float, float]:
        if self.world_pose_tractor_goal is None:
            return self.update_vw(0, 0)

        tractor_pose_goal = world_pose_tractor_est.inv().dot(
            self.world_pose_tractor_goal,
        )
        distance = np.linalg.norm(tractor_pose_goal.trans[:2])

        if distance < 0.05:
            self.world_pose_tractor_goal = None
            return self.update_vw(0, 0)
        else:
            trans_g = tractor_pose_goal.trans
            heading_error = np.arctan2(trans_g[1], trans_g[0])
            w = self.K_w * heading_error
            v = self.K_v * distance
            return self.update_vw(v, w)
