import asyncio
import logging
import sys

from farm_ng.canbus import CANSocket
from farm_ng.ipc import get_event_bus
from farm_ng.ipc import make_event
from farm_ng.motor import HubMotor
from farm_ng.periodic import Periodic
from farm_ng.steering import SteeringClient
from farm_ng.tractor.kinematics import TractorKinematics
from farm_ng.utils.proto import se3_to_proto
from farm_ng_proto.tractor.v1.geometry_pb2 import NamedSE3Pose
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp
from liegroups import SE3

logger = logging.getLogger('tractor.driver')
logger.setLevel(logging.INFO)

kinematics = TractorKinematics()


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

        self.odom_pose_tractor = SE3.identity()

        radius = (17*0.0254)/2.0  # in meters, 15" diameter wheels
        gear_ratio = 29.9
        poll_pairs = 8
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
                '\nright motor:\n  %s\nleft motor:\n  %s\n odom_pose_tractor %s left_vel %f right_vel %f',
                MessageToString(self.right_motor.get_state(), as_one_line=True),
                MessageToString(self.left_motor.get_state(), as_one_line=True),
                self.odom_pose_tractor, self._left_vel, self._right_vel,
            )

        self._left_vel = self.left_motor.average_velocity()
        self._right_vel = self.right_motor.average_velocity()
        if self._last_odom_stamp is not None:
            dt = (now.ToMicroseconds() - self._last_odom_stamp.ToMicroseconds())*1e-6
            assert dt > 0.0
            self.odom_pose_tractor = kinematics.evolve_world_pose_tractor(
                self.odom_pose_tractor,
                self._left_vel,
                self._right_vel,
                dt,
            )
            pose_msg = NamedSE3Pose()
            pose_msg.a_pose_b = se3_to_proto(self.odom_pose_tractor)
            pose_msg.frame_a = 'odometry/wheel'
            pose_msg.frame_b = 'tractor/base'
            self.event_bus.send(make_event('pose/tractor/base', pose_msg, stamp=now))

        self._last_odom_stamp = now

        self.n_cycle += 1
        brake_current = 10.0
        steering_command = self.steering.get_steering_command()
        if steering_command.brake > 0.0:
            self.right_motor.send_current_brake_command(brake_current)
            self.left_motor.send_current_brake_command(brake_current)
            self.speed = 0.0
            self.angular = 0.0
            return

        alpha = 0.1
        self.speed = self.speed * (1-alpha) + steering_command.velocity*alpha
        self.angular = self.angular * (1-alpha) + steering_command.angular_velocity*alpha

        left, right = kinematics.unicycle_to_wheel_velocity(self.speed, self.angular)
        # print(left,right)
        self.right_motor.send_velocity_command(right)
        self.left_motor.send_velocity_command(left)


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    get_event_bus('farm_ng.tractor.driver')
    controller = TractorController(event_loop)
    logger.info('Created controller %s', controller)
    event_loop.run_forever()


if __name__ == '__main__':
    main()
