import asyncio
import bisect
import logging
import sys
from collections import deque

import numpy as np
from farm_ng.canbus import CANSocket
from farm_ng.config import saved_config
from farm_ng.controller import TractorMoveToGoalController
from farm_ng.ipc import get_event_bus
from farm_ng.ipc import make_event
from farm_ng.kinematics import TractorKinematics
from farm_ng.motor import HubMotor
from farm_ng.periodic import Periodic
from farm_ng.proto_utils import proto_to_se3
from farm_ng.proto_utils import se3_to_proto
from farm_ng.steering import SteeringClient
from farm_ng_proto.tractor.v1.geometry_pb2 import NamedSE3Pose
from farm_ng_proto.tractor.v1.steering_pb2 import SteeringCommand
from farm_ng_proto.tractor.v1.tractor_pb2 import TractorState
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp
from liegroups import SE3

logger = logging.getLogger('tractor')
logger.setLevel(logging.INFO)


class TimeSeriesItem:
    def __init__(self, stamp, message):
        self.message = message
        self.stamp = stamp

    def __lt__(self, other):
        return int(self.stamp.ToMicroseconds()) < int(other.stamp.ToMicroseconds())


class TimeSeries:
    def __init__(self, time_window=1.0):
        self._items = deque()
        self._time_window = time_window

    def push(self, message, stamp):
        bisect.insort(self._items, TimeSeriesItem(stamp, message))
        if(self._items[-1].stamp.ToMicroseconds() - self._items[0].stamp.ToMicroseconds())*1e-6 > self._time_window:
            self._items.popleft()

    def find_nearest(self, stamp):
        item = self._items[bisect.bisect_left(self._items, TimeSeriesItem(stamp, None))-1]
        return item.message, item.stamp


class TractorController:
    def __init__(self, event_bus):
        self.event_loop = asyncio.get_event_loop()
        self.command_rate_hz = 50
        self.command_period_seconds = 1.0 / self.command_rate_hz
        self.n_cycle = 0

        # self.record_counter = 0
        # self.recording = False
        self.event_bus = event_bus
        self.event_bus.add_event_callback(self._on_event)

        self.lock_out = False
        self.can_socket = CANSocket('can0', self.event_loop)
        self.steering = SteeringClient(self.event_bus)
        self.tractor_state = TractorState()

        self.odom_poses_tractor = TimeSeries(1.0)
        self.odom_pose_tractor = SE3.identity()

        self.config = saved_config()

        self.kinematics = TractorKinematics(tractor_config=self.config)
        self.move_to_goal_controller = TractorMoveToGoalController()

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

    def _on_event(self, event):
        if event.name == 'pose/tractor/base/goal':
            pose = NamedSE3Pose()
            event.data.Unpack(pose)
            odom_pose_tractor, stamp = self.odom_poses_tractor.find_nearest(event.stamp)
            assert pose.frame_a == 'tractor/base'
            tractor_pose_goal = proto_to_se3(pose.a_pose_b)
            odom_pose_goal = odom_pose_tractor.dot(tractor_pose_goal)
            self.move_to_goal_controller.set_goal(odom_pose_goal)

    def _command_velocity(self, v, w):
        self.tractor_state.target_unicycle_velocity = v
        self.tractor_state.target_unicycle_angular_velocity = w
        left, right = self.kinematics.unicycle_to_wheel_velocity(
            self.tractor_state.target_unicycle_velocity,
            self.tractor_state.target_unicycle_angular_velocity,
        )
        self.tractor_state.commanded_brake_current = 0
        self.tractor_state.commanded_wheel_velocity_rads_left = left
        self.tractor_state.commanded_wheel_velocity_rads_right = right

        self.right_motor.send_velocity_command_rads(right)
        self.left_motor.send_velocity_command_rads(left)

    def _servo(self):
        v, w = self.move_to_goal_controller.update(self.odom_pose_tractor)
        self._command_velocity(v, w)

    def _command_loop(self, n_periods):
        # n_periods is the number of periods since the last call. Should normally be 1.
        now = Timestamp()
        now.GetCurrentTime()

        if (self.n_cycle % (5*self.command_rate_hz)) == 0:
            logger.info(
                '\nright motor:\n  %s\nleft motor:\n  %s\n state:\n %s',
                MessageToString(self.right_motor.get_state(), as_one_line=True),
                MessageToString(self.left_motor.get_state(), as_one_line=True),
                MessageToString(self.tractor_state, as_one_line=True),
            )

        self.tractor_state.stamp.CopyFrom(now)
        self.tractor_state.wheel_velocity_rads_left = self.left_motor.velocity_rads()
        self.tractor_state.wheel_velocity_rads_right = self.right_motor.velocity_rads()
        self.tractor_state.average_update_rate_left_motor = self.left_motor.average_update_rate()
        self.tractor_state.average_update_rate_right_motor = self.right_motor.average_update_rate()

        if self._last_odom_stamp is not None:
            dt = (now.ToMicroseconds() - self._last_odom_stamp.ToMicroseconds())*1e-6
            min_dt = 0.0
            max_dt = 1.0  # 1 second
            if dt < min_dt or dt > max_dt:
                # this condition can occur if n_periods skipped is high
                # or negative if for some reason the clock is non-monotonic - TODO(ethanrublee) should we use a monotonic clock?
                logger.warn('odometry time delta out of bounds, clipping. n_period=%d dt=%f min_dt=%f max_dt=%f', n_periods, dt, min_dt, max_dt)

            self.tractor_state.dt = np.clip(dt, min_dt, max_dt)

            tractor_pose_delta = self.kinematics.compute_tractor_pose_delta(
                self.tractor_state.wheel_velocity_rads_left,
                self.tractor_state.wheel_velocity_rads_right,
                self.tractor_state.dt,
            )

            self.odom_pose_tractor = self.odom_pose_tractor.dot(tractor_pose_delta)
            self.odom_poses_tractor.push(self.odom_pose_tractor, now)
            self.tractor_state.abs_distance_traveled += np.linalg.norm(tractor_pose_delta.trans)

            self.tractor_state.odometry_pose_base.a_pose_b.CopyFrom(se3_to_proto(self.odom_pose_tractor))
            self.tractor_state.odometry_pose_base.frame_a = 'odometry/wheel'
            self.tractor_state.odometry_pose_base.frame_b = 'tractor/base'
            self.event_bus.send(
                make_event(
                    'pose/tractor/base',
                    self.tractor_state.odometry_pose_base, stamp=now,
                ),
            )

        self._last_odom_stamp = now

        self.n_cycle += 1
        brake_current = 10.0
        steering_command = self.steering.get_steering_command()
        if steering_command.brake > 0.0:
            self.tractor_state.commanded_brake_current = brake_current
            self.tractor_state.commanded_wheel_velocity_rads_left = 0.0
            self.tractor_state.commanded_wheel_velocity_rads_right = 0.0
            self.tractor_state.target_unicycle_velocity = 0.0
            self.tractor_state.target_unicycle_angular_velocity = 0.0
            self.right_motor.send_current_brake_command(brake_current)
            self.left_motor.send_current_brake_command(brake_current)
            self.move_to_goal_controller.reset()
        elif steering_command.mode in (SteeringCommand.MODE_SERVO,):
            self._servo()
        elif steering_command.mode in (SteeringCommand.MODE_JOYSTICK_MANUAL, SteeringCommand.MODE_JOYSTICK_CRUISE_CONTROL):
            self._command_velocity(steering_command.velocity, steering_command.angular_velocity)
        self.event_bus.send(make_event('tractor_state', self.tractor_state))


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)

    event_bus = get_event_bus('farm_ng.tractor')
    controller = TractorController(event_bus)
    logger.info('Created controller %s', controller)
    event_bus.event_loop().run_forever()


if __name__ == '__main__':
    main()
