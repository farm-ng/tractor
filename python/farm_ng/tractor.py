import asyncio
import logging
import sys

import numpy as np
from farm_ng.canbus import CANSocket
from farm_ng.config import default_config
from farm_ng.ipc import get_event_bus
from farm_ng.ipc import make_event
from farm_ng.kinematics import TractorKinematics
from farm_ng.motor import HubMotor
from farm_ng.periodic import Periodic
from farm_ng.proto_utils import se3_to_proto
from farm_ng.steering import SteeringClient
from farm_ng_proto.tractor.v1.geometry_pb2 import NamedSE3Pose
from farm_ng_proto.tractor.v1.tractor_pb2 import TractorState
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp
from liegroups import SE3

logger = logging.getLogger('tractor')
logger.setLevel(logging.INFO)


class TractorController:
    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.command_rate_hz = 50
        self.command_period_seconds = 1.0 / self.command_rate_hz
        self.n_cycle = 0
        self.speed = 0.0
        self.angular = 0.0
        # self.record_counter = 0
        # self.recording = False
        self.event_bus = get_event_bus()
        self.lock_out = False
        self.can_socket = CANSocket('can0', self.event_loop)
        self.steering = SteeringClient()
        self.tractor_state = TractorState()

        self.odom_pose_tractor = SE3.identity()
        # TODO(ethanrublee|isherman) load from disk somehow.
        self.config = default_config()

        self.kinematics = TractorKinematics(tractor_config=self.config)
        radius = self.config.wheel_radius.value
        gear_ratio = self.config.hub_motor_gear_ratio.value
        poll_pairs = self.config.hub_motor_poll_pairs.value
        self.right_motor = HubMotor(
            'right_motor',
            radius, gear_ratio, poll_pairs, 7, self.can_socket,
        )
        self.left_motor = HubMotor(
            'left_motor',
            radius, gear_ratio, poll_pairs, 9, self.can_socket,
        )

        self.control_timer = Periodic(
            self.command_period_seconds, self.event_loop,
            self._command_loop, name='control_loop',
        )

        self._last_odom_stamp = None
        self._left_vel = 0.0
        self._right_vel = 0.0

    def _command_loop(self, n_periods):
        now = Timestamp()
        now.GetCurrentTime()

        if (self.n_cycle % (5*self.command_rate_hz)) == 0:
            logger.info(
                '\nright motor:\n  %s\nleft motor:\n  %s\n odom_pose_tractor %s left_vel %f right_vel %f \nstate: %s',
                MessageToString(self.right_motor.get_state(), as_one_line=True),
                MessageToString(self.left_motor.get_state(), as_one_line=True),
                self.odom_pose_tractor, self._left_vel, self._right_vel,
                MessageToString(self.tractor_state, as_one_line=True),
            )

        self._left_vel = self.left_motor.average_velocity()
        self._right_vel = self.right_motor.average_velocity()
        if self._last_odom_stamp is not None:
            dt = (now.ToMicroseconds() - self._last_odom_stamp.ToMicroseconds())*1e-6
            assert dt > 0.0

            tractor_pose_delta = self.kinematics.compute_tractor_pose_delta(
                self._left_vel,
                self._right_vel,
                dt,
            )
            self.odom_pose_tractor = self.odom_pose_tractor.dot(tractor_pose_delta)
            self.tractor_state.abs_distance_traveled += np.linalg.norm(tractor_pose_delta.trans)
            pose_msg = NamedSE3Pose()
            pose_msg.a_pose_b.CopyFrom(se3_to_proto(self.odom_pose_tractor))
            pose_msg.frame_a = 'odometry/wheel'
            pose_msg.frame_b = 'tractor/base'
            self.event_bus.send(make_event('pose/tractor/base', pose_msg, stamp=now))

        self.event_bus.send(make_event('tractor_state', self.tractor_state))
        self._last_odom_stamp = now

        self.n_cycle += 1
        brake_current = 10.0
        steering_command = self.steering.get_steering_command()
        if steering_command.brake > 0.0:
            self.right_motor.send_current_brake_command(brake_current)
            self.left_motor.send_current_brake_command(brake_current)
            self.speed = 0.0
            self.angular = 0.0
        else:
            alpha = 0.1
            self.speed = self.speed * (1-alpha) + steering_command.velocity*alpha
            self.angular = self.angular * (1-alpha) + steering_command.angular_velocity*alpha

            left, right = self.kinematics.unicycle_to_wheel_velocity(self.speed, self.angular)
            self.right_motor.send_velocity_command(right)
            self.left_motor.send_velocity_command(left)


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    get_event_bus('farm_ng.tractor')
    controller = TractorController(event_loop)
    logger.info('Created controller %s', controller)
    event_loop.run_forever()


if __name__ == '__main__':
    main()
