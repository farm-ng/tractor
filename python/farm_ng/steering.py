import time
import asyncio
import logging
from farm_ng.periodic import Periodic
from farm_ng_proto.tractor.v1.steering_pb2 import SteeringCommand
from google.protobuf.text_format import MessageToString
from farm_ng.ipc import get_event_bus, make_event

logger = logging.getLogger('steering')
logger.setLevel(logging.INFO)

event_bus = get_event_bus()


_g_message_name='steering'

class SteeringClient(object):
    def __init__(self):
        self._latest_command = SteeringCommand()
        self._stop_command = SteeringCommand()
        self._stop_command.deadman = 0.0
        self._stop_command.brake = 1.0
        self._stop_command.velocity = 0.0
        self._stop_command.angular_velocity = 0.0
        self.lockout = True
        
        
    def get_steering_command(self):
        event = event_bus.get_last_event(_g_message_name)
        if event is None:
            self.lockout = True
            return self._stop_command

        if (time.time()*1000.0 - event.stamp.ToMilliseconds() > 200):
            self.lockout = True
            return self._stop_command

        event.data.Unpack(self._latest_command)

        if self.lockout is True:
            if abs(self._latest_command.velocity) > 0.01 or abs(self._latest_command.angular_velocity) > 0.01:
                return self._stop_command
            self.lockout = False
            
        return self._latest_command        

class SteeringSenderJoystick(object):
    def __init__(self):
        loop = asyncio.get_event_loop()
        self.joystick = MaybeJoystick('/dev/input/js0',  loop)
        self.rate_hz = 50.0
        self.period = 1.0/self.rate_hz
        self._periodic = Periodic(self.period, loop, self.send)
        self._command = SteeringCommand()

    def send(self, n_periods):
        if not self.joystick.get_button_state('tl2', False) or not self.joystick.is_connected() or n_periods > self.rate_hz/4:
            self._command.velocity = 0.0
            self._command.angular_velocity = 0.0
            self._command.brake = 1.0
            self._command.deadman = 0.0
        else:
            self._command.deadman = 1.0 if self.joystick.get_button_state('tl2', False) else 0.0
            self._command.brake = 0.0
            
            velocity = np.clip(-self.joystick.get_axis_state('y', 0), -1.0, 1.0)
            if abs(velocity) < 0.5:
                velocity = velocity/4.0
            if abs(velocity) >= 0.5:
                velocity = np.sign(velocity) *(0.5/4 + (abs(velocity) - 0.5)*2)
            self._command.velocity = velocity
            
            angular_velocity = np.clip(-self.joystick.get_axis_state('rx', 0), -1.0, 1.0)*np.pi/3.0
            self._command.angular_velocity = angular_velocity
        event_loop.send(make_event(_g_message_name, self._command))

def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    steering_sender = SteeringSenderJoystick()
    event_loop.run_forever()

if __name__ == '__main__':
    main()
