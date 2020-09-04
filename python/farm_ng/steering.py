import asyncio
import logging
import sys
import time

import numpy as np
from farm_ng.ipc import get_event_bus
from farm_ng.ipc import make_event
from farm_ng.joystick import MaybeJoystick
from farm_ng.periodic import Periodic
from farm_ng_proto.tractor.v1.steering_pb2 import SteeringCommand

logger = logging.getLogger('steering')
logger.setLevel(logging.INFO)


_g_message_name = 'steering'


class SteeringClient:
    def __init__(self):
        self._latest_command = SteeringCommand()
        self._stop_command = SteeringCommand()
        self._stop_command.deadman = 0.0
        self._stop_command.brake = 1.0
        self._stop_command.velocity = 0.0
        self._stop_command.angular_velocity = 0.0
        self.lockout = True

    def get_steering_command(self):
        event = get_event_bus('steering').get_last_event(_g_message_name)
        if event is None:
            self.lockout = True
            return self._stop_command

        delta_t_millisecond = time.time()*1000.0 - event.recv_stamp.ToMilliseconds()
        if (delta_t_millisecond > 1000):
            logger.warning('steering lock out due to long time since last event: %d' % delta_t_millisecond)
            self.lockout = True
            return self._stop_command

        event.data.Unpack(self._latest_command)

        if self.lockout is True:
            if abs(self._latest_command.velocity) > 0.01 or abs(self._latest_command.angular_velocity) > 0.01:
                return self._stop_command
            self.lockout = False

        return self._latest_command


class SteeringSenderJoystick:
    def __init__(self):
        loop = asyncio.get_event_loop()
        self.rate_hz = 50.0
        self.period = 1.0/self.rate_hz
        self._executing_motion_primitive = False
        self._delta_x_vel = 0.25/self.rate_hz
        self._delta_angular_vel = np.pi/16

        self.joystick = MaybeJoystick('/dev/input/js0',  loop)
        self.joystick.set_button_callback(self.on_button)
        self._periodic = Periodic(self.period, loop, self.send)
        self._command = SteeringCommand()

    def stop(self):
        self._command.velocity = 0.0
        self._command.angular_velocity = 0.0
        self._command.brake = 1.0
        self._command.deadman = 0.0
        self._executing_motion_primitive = False

    def on_button(self, button, value):
        if button == 'touch' and value:
            self.stop()

    def send(self, n_periods):
        if self.joystick.get_axis_state('hat0y', 0.0) != 0:
            self._executing_motion_primitive = True
            # hat0y -1 is up dpad
            # hat0y +1 is down dpad
            inc_vel = -self.joystick.get_axis_state('hat0y', 0.0)*self._delta_x_vel
            self._command.brake = 0.0
            self._command.deadman = 0.0
            self._command.angular_velocity = 0.0
            self._command.velocity = np.clip(
                self._command.velocity + inc_vel,
                -2, 2,
            )

        if self.joystick.get_axis_state('hat0x', 0.0) != 0:
            self._executing_motion_primitive = True
            # hat0x -1 is left dpad
            # hat0x +1 is right dpad
            inc_vel = -self.joystick.get_axis_state('hat0x', 0.0)*self._delta_angular_vel
            self._command.brake = 0.0
            self._command.deadman = 0.0
            self._command.angular_velocity = inc_vel
        elif self._executing_motion_primitive:
            # angular velocity resets if left or right dpad is not pressed,
            # so its like a nudge rather than cruise control.
            self._command.angular_velocity = 0.0

        if not self._executing_motion_primitive:
            if not self.joystick.get_button_state('L2', False) or not self.joystick.is_connected() or n_periods > self.rate_hz/4:
                self.stop()
            else:
                self._command.deadman = 1.0 if self.joystick.get_button_state('L2', False) else 0.0
                self._command.brake = 0.0

                velocity = np.clip(-self.joystick.get_axis_state('y', 0), -1.0, 1.0)
                if abs(velocity) < 0.5:
                    velocity = velocity/4.0
                if abs(velocity) >= 0.5:
                    velocity = np.sign(velocity) * (0.5/4 + (abs(velocity) - 0.5)*2)
                self._command.velocity = velocity
                angular_velocity = np.clip(-self.joystick.get_axis_state('z', 0), -1.0, 1.0)*np.pi/3.0
                self._command.angular_velocity = angular_velocity

        get_event_bus('steering').send(make_event(_g_message_name, self._command))


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    _ = SteeringSenderJoystick()
    event_loop.run_forever()


if __name__ == '__main__':
    main()
