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

        # whether the tractor is executing a motion primitive (e.g. cruise control, indexing)
        self._executing_motion_primitive = False

        # rate that the speed of the tractor changes when pressing the dpad up/down arrows
        self._delta_x_vel = 0.25/self.rate_hz
        # rate of angular velocity that occurs when nudging the tractor left/right with dpad left/right arrows
        self._delta_angular_vel = np.pi/16
        
        # maximum acceleration for manual steering and motion primitives
        self._max_acc = 1.5/self.rate_hz # m/s^2
        # The target speed, used for smoothing out the commanded velocity to respect the max_acc limit
        self._target_speed = 0.0

        self.joystick = MaybeJoystick('/dev/input/js0',  loop)
        self.joystick.set_button_callback(self.on_button)
        self._periodic = Periodic(self.period, loop, self.send)
        self._command = SteeringCommand()


        # indexing state machine variables
        # ammount of time spent moving during indexing, if None, will move forever
        self._moving_duration = None
        # amount of time spent stopping during indexing, if None, will stop forever
        self._stopping_duration = None
        # whether we are indexing or not
        self._indexing = False
        # Indexing state machine state, can be one of 'starting', 'moving', 'stopping'
        self._indexing_state = 'starting'
        # The last set indexing speed, to be applied while in 'moving' state
        self._indexing_speed = 0.0
        # When moving started
        self._moving_start_time = None
        # When stopping started
        self._stopping_start_time = None

    def start_motion_primitive(self):
        self._executing_motion_primitive = True
        self._command.brake = 0.0
        self._command.deadman = 0.0

    def stop(self):
        self._command.velocity = 0.0
        self._command.angular_velocity = 0.0
        self._command.brake = 1.0
        self._target_speed = 0.0
        self._command.deadman = 0.0
        self._indexing = False
        self._executing_motion_primitive = False
        
    def on_button(self, button, value):
        if button == 'touch' and value:
            self.stop()


        if button == "cross" and value:
            # Pressing cross causes the indexing behavor to toggle on/off.
            # Pressing cross will recall the last indexing speed and moving/stopping durations.
            self._indexing = not self._indexing
            if not self._indexing:
                self.stop()
            else:
                self.enter_indexing_state('starting')
                
        if button == 'square' and value:
            # pressing 'square' causes the indexing behavior to stop
            # and reset indexing moving/stopping durations and speed settings
            self.stop()
            self._indexing_speed = 0.0
            self._moving_duration = None
            self._stopping_duration = None

        if button == 'triangle' and self._indexing:
            # Measures the time that the triangle button is pressed, and sets the moving duration to the measured time.
            # The steering enteres the moving state, so the tractor may move (if there is an indexing_speed set)
            if value:
                self.enter_indexing_state('moving')
                self._moving_duration = None
            else:
                self._moving_duration = time.time() - self._moving_start_time
                print('index duration', self._moving_duration)

        if button == 'circle' and self._indexing:
            # Measures the time that the circle button is pressed, and sets the stopping duration to the measured time.
            # This also causes the steering to enter the stopping state, so the tractor stops while the circle button is pressed.
            if value:
                self.enter_indexing_state('stopping')
                self._stopping_duration = None
            else:
                self._stopping_duration = time.time() - self._stopping_start_time
                print('pause duration: ', self._stopping_duration)
            
        
    def enter_indexing_state(self, state_name):
        self._indexing_state = state_name
        if self._indexing_state == 'starting':
            self.start_motion_primitive()
        if self._indexing_state == 'moving':
            self._target_speed = self._indexing_speed
            self._moving_start_time = time.time()
        if self._indexing_state == 'stopping':
            self._stopping_start_time = time.time()
            self._target_speed = 0.0
        

    def send(self, n_periods):
        if self.joystick.get_axis_state('hat0y', 0.0) != 0:
            self.start_motion_primitive()
            # hat0y -1 is up dpad
            # hat0y +1 is down dpad
            inc_vel = -self.joystick.get_axis_state('hat0y', 0.0)*self._delta_x_vel
            self._command.angular_velocity = 0.0
                        
            self._target_speed = np.clip(
                self._target_speed + inc_vel,
                -2, 2,
            )
            if self._indexing and self._indexing_state == 'moving':
                self._indexing_speed = self._target_speed

        if self.joystick.get_axis_state('hat0x', 0.0) != 0:
            self.start_motion_primitive()
            # hat0x -1 is left dpad
            # hat0x +1 is right dpad
            angular_vel = -self.joystick.get_axis_state('hat0x', 0.0)*self._delta_angular_vel
            self._command.angular_velocity = angular_vel

        elif self._executing_motion_primitive:
            # angular velocity resets if left or right dpad is not pressed,
            # so its like a nudge rather than cruise control.
            self._command.angular_velocity = 0.0

        if self._executing_motion_primitive and self._indexing:
            if self._indexing_state == 'starting':
                self.enter_indexing_state('moving')
  
            elif self._indexing_state == 'moving':
                self._target_speed = self._indexing_speed
                if self._moving_duration is not None and time.time() - self._moving_start_time > self._moving_duration:
                    self.enter_indexing_state('stopping')

            elif self._indexing_state == 'stopping':
                self._target_speed = 0.0                
                if self._stopping_duration is not None and (time.time() - self._stopping_start_time) > self._stopping_duration:
                    self.enter_indexing_state('moving')
                
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
                self._target_speed = velocity
                angular_velocity = np.clip(-self.joystick.get_axis_state('z', 0), -1.0, 1.0)*np.pi/3.0
                self._command.angular_velocity = angular_velocity

        self._command.velocity += np.clip((self._target_speed - self._command.velocity), -self._max_acc, self._max_acc)
        get_event_bus('steering').send(make_event(_g_message_name, self._command))


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    _ = SteeringSenderJoystick()
    event_loop.run_forever()


if __name__ == '__main__':
    main()
