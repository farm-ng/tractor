import asyncio
import logging
import math
import sys

import farm_ng.proio_utils
from farm_ng.canbus import CANSocket
from farm_ng.joystick import MaybeJoystick
from farm_ng.motor import HubMotor
from farm_ng.periodic import Periodic
from google.protobuf.text_format import MessageToString
from farm_ng.tractor.kinematics import TractorKinematics
import numpy as np

logger = logging.getLogger('tractor.driver')
plog = farm_ng.proio_utils.get_proio_logger()

kinematics = TractorKinematics()

class TractorController:
    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.command_rate_hz = 50
        self.command_period_seconds = 1.0 / self.command_rate_hz
        self.n_cycle = 0
        self.speed = 0.0
        self.angular = 0.0
        self.lock_out = False
        self.can_socket = CANSocket('can0', self.event_loop)
        self.joystick = MaybeJoystick('/dev/input/js0', self.event_loop)

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
            self._command_loop,
        )

    def _command_loop(self, n_periods):
        if (self.n_cycle % (2*self.command_rate_hz)) == 0:
            logger.info('right motor: %s', MessageToString(self.right_motor.get_state(), as_one_line=True))
            logger.info('left motor: %s', MessageToString(self.left_motor.get_state(), as_one_line=True))
        self.n_cycle += 1

        brake_current=10.0
        
        if ( n_periods*self.command_period_seconds >= 0.25 or
             not self.joystick.is_connected() or self.joystick.get_axis_state('rx', -1) < 0.0):
            self.speed = 0.0
            self.angular = 0.0
            self.right_motor.send_current_brake_command(brake_current)
            self.left_motor.send_current_brake_command(brake_current)                
            self.lock_out = True
            return
        else:
            speed = np.clip(-self.joystick.get_axis_state('y', 0), -1.0, 1.0)
            angular = np.clip(-self.joystick.get_axis_state('z', 0), -1.0, 1.0)
            if self.lock_out and (np.abs(speed) > 0.1 or np.abs(angular) > 0.1):
                speed = 0.0
                angular = 0.0
            else:
                self.lock_out = False
            speed = 1.5*speed
            angular = angular*np.pi/3.0
        alpha = 0.02
        delta_speed = np.clip(speed - self.speed, -alpha, alpha)
        delta_angular = np.clip(angular - self.angular, -alpha, alpha)
        self.speed = np.clip(self.speed + delta_speed, -1.5, 1.5)
        self.angular = np.clip(self.angular + delta_angular, -np.pi/3.0, np.pi/3.0)
        left, right = kinematics.unicycle_to_wheel_velocity(self.speed, self.angular)
        self.right_motor.send_velocity_command(right)
        self.left_motor.send_velocity_command(left)


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    controller = TractorController(event_loop)
    logger.info('Created controller %s', controller)
    _ = Periodic(60, event_loop, lambda n_periods: plog.writer().flush())
    event_loop.run_forever()


if __name__ == '__main__':
    main()
