import logging
import sys
import time

import numpy as np
from farm_ng.ipc import EventBus
from farm_ng.ipc import get_event_bus
from farm_ng.ipc import make_event
from farm_ng.ipc import Subscription
from farm_ng.joystick import MaybeJoystick
from farm_ng.periodic import Periodic
from farm_ng_proto.tractor.v1.steering_pb2 import SteeringCommand

logger = logging.getLogger('steering')
logger.setLevel(logging.INFO)


_g_message_name = 'steering'


def SetStopCommand(command):
    command.deadman = 0.0
    command.brake = 1.0
    command.velocity = 0.0
    command.angular_velocity = 0.0


class SteeringClient:
    def __init__(self, event_bus):
        self._latest_command = SteeringCommand()
        self._stop_command = SteeringCommand()
        SetStopCommand(self._stop_command)
        self.lockout = True
        self._event_bus = event_bus
        self._event_bus.add_subscriptions([Subscription(name=_g_message_name)])

    def get_steering_command(self):
        event = self._event_bus.get_last_event(_g_message_name)
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


class BaseSteering:
    def __init__(self, rate_hz, mode):
        self.rate_hz = rate_hz
        self.v = 0
        self.v_acc = 2.0/self.rate_hz
        self.v_max = 2.0
        self.w = 0
        self.w_acc = (2*np.pi)/self.rate_hz
        self.w_max = np.pi/2
        self.gamma = 2.5
        self.command = SteeringCommand()
        self.command.mode = mode
        self.stop()

    def stop(self):
        self.v = 0
        self.w = 0
        SetStopCommand(self.command)

    def update_vw(self, v, w):
        self.v += np.clip(v - self.v, -self.v_acc, self.v_acc)
        self.v = np.clip(self.v, -self.v_max, self.v_max)

        self.w += np.clip(w - self.w, -self.w_acc, self.w_acc)
        self.w = np.clip(self.w, -self.w_max, self.w_max)
        self.command.velocity = self.v
        self.command.angular_velocity = self.w


class JoystickManualSteering(BaseSteering):
    def __init__(self, rate_hz, joystick):
        super().__init__(rate_hz, SteeringCommand.MODE_JOYSTICK_MANUAL)
        self.joystick = joystick
        self.gamma = 2.5
        self.target_gas = 0.0
        self.target_steering = 0.0

    def stop(self):
        super().stop()
        self.target_gas = 0.0
        self.target_steering = 0.0

    def update(self):
        if not self.joystick.get_button_state('L2', False) or not self.joystick.is_connected():
            self.stop()
        else:
            self.command.deadman = 1.0 if self.joystick.get_button_state('L2', False) else 0.0
            self.command.brake = 0.0
            gas = np.clip(-self.joystick.get_axis_state('y', 0), -1.0, 1.0)
            gas = np.sign(gas)*abs(gas)**(self.gamma)
            alpha = 0.5
            self.target_gas = (1.0-alpha)*self.target_gas + alpha*gas
            steering = np.clip(-self.joystick.get_axis_state('z', 0), -1.0, 1.0)
            steering = np.sign(steering)*abs(steering)**(self.gamma)
            self.target_steering = (1.0-alpha)*self.target_steering + alpha*steering
            self.update_vw(self.v_max*self.target_gas, self.w_max*self.target_steering)
        return self.command


class CruiseControlSteering(BaseSteering):
    def __init__(self, rate_hz, joystick):
        super().__init__(rate_hz, SteeringCommand.MODE_JOYSTICK_CRUISE_CONTROL)
        self.joystick = joystick
        # rate that the speed of the tractor changes when pressing the dpad up/down arrows
        self.delta_x_vel = 0.25/self.rate_hz
        # rate of angular velocity that occurs when nudging the tractor left/right with dpad left/right arrows
        self.delta_angular_vel = np.pi/6

        # The target speed for cruise control
        self.target_speed = 0.0
        self.target_angular_velocity = 0.0

    def stop(self):
        self.target_speed = 0.0
        self.target_angular_velocity = 0.0
        super().stop()

    def cruise_control_axis_active(self):
        return self.joystick.get_axis_state('hat0y', 0.0) != 0 or self.joystick.get_axis_state('hat0x', 0.0) != 0

    def update(self):
        self.command.brake = 0.0
        self.command.deadman = 0.0
        if self.joystick.get_axis_state('hat0y', 0.0) != 0:
            # hat0y -1 is up dpad
            # hat0y +1 is down dpad
            inc_vel = -self.joystick.get_axis_state('hat0y', 0.0)*self.delta_x_vel
            self.target_angular_velocity = 0.0

            self.target_speed = np.clip(
                self.target_speed + inc_vel,
                -self.v_max, self.v_max,
            )

        if self.joystick.get_axis_state('hat0x', 0.0) != 0:
            # hat0x -1 is left dpad
            # hat0x +1 is right dpad
            angular_vel = -self.joystick.get_axis_state('hat0x', 0.0)*self.delta_angular_vel
            self.target_angular_velocity = angular_vel
        else:
            # angular velocity resets if left or right dpad is not pressed,
            # so its like a nudge rather than cruise control.
            self.target_angular_velocity = 0.0

        self.update_vw(self.target_speed, self.target_angular_velocity)
        return self.command


class SteeringSenderJoystick:
    def __init__(self, event_bus: EventBus):
        self._event_bus = event_bus
        self.rate_hz = 50.0
        self.period = 1.0/self.rate_hz

        # whether the tractor is executing a motion primitive (e.g. cruise control, indexing)
        self._executing_motion_primitive = False

        self.joystick = MaybeJoystick('/dev/input/js0',  self._event_bus.event_loop())
        self.joystick.set_button_callback(self.on_button)
        self._periodic = Periodic(self.period, self._event_bus.event_loop(), self.send)

        self.joystick_manual_steer = JoystickManualSteering(self.rate_hz, self.joystick)
        self.cruise_control_steer = CruiseControlSteering(self.rate_hz, self.joystick)
        self.cruise_control_active = False

        self.stop()

    def _start_cruise_control(self):
        if not self.cruise_control_active:
            self.cruise_control_steer.command.velocity = self.joystick_manual_steer.command.velocity
            self.cruise_control_steer.command.angular_velocity = self.joystick_manual_steer.command.angular_velocity
        self.cruise_control_active = True

    def stop(self):
        self.cruise_control_active = False
        self.joystick_manual_steer.stop()
        self.cruise_control_steer.stop()

    def on_button(self, button, value):
        if button == 'touch' and value:
            self.stop()

        if button in ('hat0x', 'hat0y') and value:
            self._start_cruise_control()

    def send(self, n_periods):
        if n_periods > self.rate_hz:
            self.stop()
            command = self.joystick_manual_steer.command

        if self.cruise_control_steer.cruise_control_axis_active():
            self._start_cruise_control()

        if self.cruise_control_active:
            command = self.cruise_control_steer.update()
        else:
            command = self.joystick_manual_steer.update()
        self._event_bus.send(make_event(_g_message_name, command))


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_bus = get_event_bus('steering')
    _ = SteeringSenderJoystick(event_bus)
    event_bus.event_loop().run_forever()


if __name__ == '__main__':
    main()
