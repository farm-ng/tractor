import asyncio
import logging
import sys

from farm_ng.ipc import get_event_bus
from farm_ng.ipc import make_event
from farm_ng.periodic import Periodic
from farm_ng.tractor.kinematics import TractorKinematics
from farm_ng.utils.proto import se3_to_proto
from farm_ng_proto.tractor.v1.geometry_pb2 import NamedSE3Pose
from liegroups import SE3

logger = logging.getLogger('pose_vis_toy')
logger.setLevel(logging.INFO)


class PoseVisToy:
    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.command_rate_hz = 50
        self.command_period_seconds = 1.0 / self.command_rate_hz
        self.event_bus = get_event_bus('farm_ng.pose_vis_toy')
        self.odom_pose_tractor = SE3.identity()
        self.kinematic_model = TractorKinematics()

        self.control_timer = Periodic(
            self.command_period_seconds, self.event_loop,
            self._command_loop, name='control_loop',
        )

    def _command_loop(self, n_periods):
        pose_msg = NamedSE3Pose()
        self.odom_pose_tractor = self.kinematic_model.evolve_world_pose_tractor(self.odom_pose_tractor, 1, 0.5, n_periods*self.command_period_seconds)
        pose_msg.a_pose_b.CopyFrom(se3_to_proto(self.odom_pose_tractor))
        pose_msg.frame_a = 'odometry/wheel'
        pose_msg.frame_b = 'tractor/base'
        self.event_bus.send(make_event('pose/tractor/base', pose_msg))


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()

    controller = PoseVisToy(event_loop)
    logger.info('Created controller %s', controller)
    event_loop.run_forever()


if __name__ == '__main__':
    main()
