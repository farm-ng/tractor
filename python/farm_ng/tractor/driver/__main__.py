import asyncio
import logging
import math
import sys

from farm_ng.canbus import CANSocket
from farm_ng.joystick import MaybeJoystick
from farm_ng.motor import HubMotor
from farm_ng.periodic import Periodic
from farm_ng.rtkcli import RtkClient

logger = logging.getLogger('tractor.driver')


def steering(x, y):
    # https://electronics.stackexchange.com/a/293108
    # convert to polar
    r = math.hypot(x, y)
    t = math.atan2(y, x)

    # rotate by 45 degrees
    t += math.pi / 4

    # back to cartesian
    left = r * math.cos(t)
    right = r * math.sin(t)

    # rescale the new coords
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)

    # clamp to -1/+1
    left = max(-1, min(left, 1))
    right = max(-1, min(right, 1))

    return left, right


class TractorController:
    def __init__(self, event_loop, rtk_client_host):
        self.event_loop = event_loop
        self.command_rate_hz = 50
        self.command_period_seconds = 1.0 / self.command_rate_hz
        self.n_cycle = 0

        self.can_socket = CANSocket('can0', self.event_loop)
        self.joystick = MaybeJoystick('/dev/input/js0', self.event_loop)

        radius = (15.0*0.0254)/2.0  # in meters, 15" diameter wheels
        gear_ratio = 6
        poll_pairs = 15
        self.right_motor = HubMotor(
            radius, gear_ratio, poll_pairs, 7, self.can_socket,
        )
        self.left_motor = HubMotor(
            radius, gear_ratio, poll_pairs, 9, self.can_socket,
        )

        self.rtk_client = RtkClient(rtk_client_host, 9797, event_loop)

        self.control_timer = Periodic(
            self.command_period_seconds, self.event_loop,
            self._command_loop,
        )

    def _command_loop(self):
        if (self.n_cycle % (2*self.command_rate_hz)) == 0:
            logger.info('right VESC: %s', self.right_motor.get_state())
            logger.info('left VESC: %s', self.left_motor.get_state())
            logger.info('gps solution: %s', self.rtk_client.gps_states[-1])
        self.n_cycle += 1

        # called once each command period
        if self.joystick.get_axis_state('brake', -1) < 0.999:
            # deadman not pushed
            if self.joystick.is_connected():
                self.right_motor.send_velocity_command(0.0)
                self.left_motor.send_velocity_command(0.0)
            else:
                self.right_motor.send_current_command(0.0)
                self.left_motor.send_current_command(0.0)
            return

        speed = -self.joystick.get_axis_state('y', 0)
        turn = -self.joystick.get_axis_state('z', 0)
        left, right = steering(speed, turn)
        self.right_motor.send_velocity_command(right)
        self.left_motor.send_velocity_command(left)


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
    event_loop = asyncio.get_event_loop()
    controller = TractorController(event_loop, 'localhost')
    logger.info('Created controller %s', controller)
    event_loop.run_forever()


if __name__ == '__main__':
    main()
