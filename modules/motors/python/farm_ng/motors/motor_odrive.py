import asyncio
import logging
import math
import socket
import struct
import sys
import time

# import can
import linuxfd
import numpy as np
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp

from farm_ng.core.ipc import get_event_bus
from farm_ng.core.ipc import make_event
from farm_ng.core.periodic import Periodic
from farm_ng.motors import motor_pb2
# from farm_ng.tractor import motor_pb2
from farm_ng.motors.canbus import CANSocket

# from farm_ng.motors.CANSimple import odrive_s

# from farm_ng.tractor.config import TractorConfigManager

logger = logging.getLogger('farm_ng.motor_odrive')

logger.setLevel(logging.INFO)


def HeartBeatToProto(fields, odrive_axis: motor_pb2.ODriveAxis):
    err_bitset = fields['error']
    del odrive_axis.error[:]
    if err_bitset != 0:
        for error in motor_pb2.ODriveAxis.Error.values():
            if err_bitset & error:
                odrive_axis.error.append(error)
    odrive_axis.current_state = fields['current_state']


def EncoderEstimatesToProto(fields, odrive_axis: motor_pb2.ODriveAxis):
    odrive_axis.encoder_position_estimate.value = fields['encoder_pos_estimate']
    odrive_axis.encoder_velocity_estimate.value = fields['encoder_vel_estimate']


def VbusVoltageToProto(fields, odrive_axis: motor_pb2.ODriveAxis):
    odrive_axis.vbus_voltage.value = fields['vbus_voltage']

# Taken from https://github.com/madcowswe/ODrive/blob/devel/tools/odrive/tests/can_test.py
#
# Each argument is described as tuple (name, format, scale).
# Struct format codes: https://docs.python.org/2/library/struct.html
command_set = {
    'heartbeat': (0x001, [('error', 'I', 1), ('current_state', 'I', 1)], HeartBeatToProto),  # tested
    'estop': (0x002, []),  # tested
    'get_motor_error': (0x003, [('motor_error', 'I', 1)]),  # untested
    'get_encoder_error': (0x004, [('encoder_error', 'I', 1)]),  # untested
    'get_sensorless_error': (0x005, [('sensorless_error', 'I', 1)]),  # untested
    'set_node_id': (0x006, [('node_id', 'I', 1)]),  # tested
    'set_requested_state': (0x007, [('requested_state', 'I', 1)]),  # tested
    # 0x008 not yet implemented
    'get_encoder_estimates': (0x009, [('encoder_pos_estimate', 'f', 1), ('encoder_vel_estimate', 'f', 1)], EncoderEstimatesToProto),  # partially tested
    'get_encoder_count': (0x00a, [('encoder_shadow_count', 'i', 1), ('encoder_count', 'i', 1)]),  # partially tested
    'set_controller_modes': (0x00b, [('control_mode', 'i', 1), ('input_mode', 'i', 1)]),  # tested
    'set_input_pos': (0x00c, [('input_pos', 'f', 1), ('vel_ff', 'h', 0.001), ('torque_ff', 'h', 0.001)]),  # tested
    'set_input_vel': (0x00d, [('input_vel', 'f', 1), ('torque_ff', 'f', 1)]),  # tested
    'set_input_torque': (0x00e, [('input_torque', 'f', 1)]),  # tested
    'set_velocity_limit': (0x00f, [('velocity_limit', 'f', 1)]),  # tested
    'start_anticogging': (0x010, []),  # untested
    'set_traj_vel_limit': (0x011, [('traj_vel_limit', 'f', 1)]),  # tested
    'set_traj_accel_limits': (0x012, [('traj_accel_limit', 'f', 1), ('traj_decel_limit', 'f', 1)]),  # tested
    'set_traj_inertia': (0x013, [('inertia', 'f', 1)]),  # tested
    'get_iq': (0x014, [('iq_setpoint', 'f', 1), ('iq_measured', 'f', 1)]),  # untested
    'get_sensorless_estimates': (0x015, [('sensorless_pos_estimate', 'f', 1), ('sensorless_vel_estimate', 'f', 1)]),  # untested
    'reboot': (0x016, []),  # tested
    'get_vbus_voltage': (0x017, [('vbus_voltage', 'f', 1)], VbusVoltageToProto),  # tested
    'clear_errors': (0x018, []),  # partially tested
}


class CANSimpleParser:
    def __init__(self, cmd_name, cmd_spec):
        self.cmd_name = cmd_name
        self.cmd_spec = cmd_spec
        self.cmd_id = cmd_spec[0]
        self.fmt = '<' + ''.join([f for (n, f, s) in cmd_spec[1]])  # all little endian
        self.fmt_size = struct.calcsize(self.fmt)

    def parse(self, msg, odrive_axis: motor_pb2.ODriveAxis):
        fields = struct.unpack(self.fmt, msg[:self.fmt_size])
        fields = {n: (fields[i] * s) for (i, (n, f, s)) in enumerate(self.cmd_spec[1])}
        if(len(self.cmd_spec) > 2):
            self.cmd_spec[2](fields, odrive_axis)
        return fields

    def __str__(self):
        return '%s : %04x - fmt: %s' % (self.cmd_name, self.cmd_id, self.fmt)


def make_parsers():
    parsers = dict()
    for cmd_name, cmd_spec in command_set.items():
        parser = CANSimpleParser(cmd_name, cmd_spec)
        print(parser)
        parsers[parser.cmd_id] = parser
    return parsers


g_odrive_parsers = make_parsers()


# sys.exit(0)

# https://github.com/madcowswe/ODrive/blob/devel/tools/odrive/tests/can_test.py


class HubMotor:
    def __init__(
            self, name,  can_node_id, can_socket, radius=1.0, gear_ratio=1.0, invert=False):
        self.name = name
        self.radius = radius
        self.gear_ratio = gear_ratio
        self.invert = invert
        self.can_node_id = can_node_id
        self.can_socket = can_socket
        self._event_bus = get_event_bus(self.name)
        self._latest_state = motor_pb2.ODriveAxis()
        self._request_period_seconds = 1.0/50.0
        self._request_count = 0
        self._request_timer = Periodic(
            self._request_period_seconds, self._event_bus.event_loop(),
            self._request_loop, name='%s/request_loop' % name,
        )
        self.can_socket.add_reader(self._handle_can_message)

    def _request_loop(self, n_periods):
        self.get_encoder_estimates()
        if self._request_count % 10 == 0:
            self.get_vbus_voltage()
        self._request_count += 1

    def _handle_can_message(self, cob_id, data, stamp):
        can_node_id = ((cob_id >> 5) & 0b111111)  # upper 6 bits
        if can_node_id != self.can_node_id:
            return
        command = (cob_id & 0b11111)  # lower 5 bits
        parser = g_odrive_parsers.get(command, None)
        if parser is None:
            logger.warning(
                'No parser for command :%x node id: %d', command, can_node_id,
            )
            return

        parser.parse(data, self._latest_state)

        #logger.info('can node id %02d %s', can_node_id, msg)
        self._latest_state.stamp.CopyFrom(stamp)

        if (parser.cmd_name == "get_encoder_estimates"):
            if self.invert:
                self._latest_state.encoder_position_estimate.value = -self._latest_state.encoder_position_estimate.value
                self._latest_state.encoder_velocity_estimate.value = -self._latest_state.encoder_velocity_estimate.value

            event = make_event('%s/state' % self.name, self._latest_state, stamp=self._latest_state.stamp)
            self._event_bus.send(event)

    def _send_can_command(self, cmd_name, is_request=False, **kwargs):
        cmd_spec = command_set[cmd_name]
        cmd_id = cmd_spec[0]
        fmt = '<' + ''.join([f for (n, f, s) in cmd_spec[1]])  # all little endian

        if (sorted([n for (n, f, s) in cmd_spec[1]]) != sorted(kwargs.keys())):
            raise Exception("expected arguments: " + str([n for (n, f, s) in cmd_spec[1]]))

        fields = [((kwargs[n] / s) if f == 'f' else int(kwargs[n] / s)) for (n, f, s) in cmd_spec[1]]
        data = struct.pack(fmt, *fields)

        cob_id = int(self.can_node_id << 5) | cmd_id

        self.can_socket.send(cob_id, data, flags=socket.CAN_RTR_FLAG if is_request else 0)

    def _send_can_request(self, cmd_name, **kwargs):
        cmd_spec = command_set[cmd_name]
        cmd_id = cmd_spec[0]
        cob_id = int(self.can_node_id << 5) | cmd_id
        self.can_socket.send(cob_id, bytes([]), flags=socket.CAN_RTR_FLAG)

    def get_encoder_estimates(self):
        self._send_can_request('get_encoder_estimates')
    
    def get_vbus_voltage(self):
        self._send_can_request('get_vbus_voltage')

    def set_input_velocity(self, vel):
        # vel in turns/second
        self._latest_state.input_velocity.value = vel

        if self.invert:
            vel = -vel
        self._send_can_command('set_input_vel', input_vel=vel, torque_ff=0.0)

    def set_input_velocity_rads(self, vel):
        vel_turns = vel/(2*math.pi)
        self.set_input_velocity(vel_turns*self.gear_ratio)

    def velocity_rads(self):
        return self._latest_state.input_velocity.value/self.gear_ratio * 2*math.pi

    def average_update_rate(self):
        return self._request_period_seconds

    def set_requested_state(self, state: motor_pb2.ODriveAxis.State):
        self._send_can_command('set_requested_state', requested_state=state)

    def clear_errors(self):
        self._send_can_command('clear_errors')

    def reboot(self):
        self._send_can_command('reboot')

    def get_state(self):
        return self._latest_state


def main():

    command_rate_hz = 50
    command_period_seconds = 1.0 / command_rate_hz
    # rtc=False means a monotonic clock for realtime loop as it won't
    # be adjusted by the system admin
    periodic = linuxfd.timerfd(rtc=False, nonBlocking=True)
    # here we start a timer that will fire in one second, and then
    # each command period after that
    periodic.settime(value=1.0, interval=command_period_seconds)

    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    try:
        can_socket = CANSocket('can0')
    except OSError as e:
        sys.stderr.write(
            'Could not listen on interface can0\n',
        )
        sys.exit(e.errno)

    print('Listening on can0')
    loop = asyncio.get_event_loop()

    right_motor = HubMotor(
        'right_motor',  20, can_socket,  invert=True
    )
    right_motor_aft = HubMotor(
        'right_motor_aft', 21, can_socket,  invert=True
    )
    left_motor = HubMotor(
        'left_motor', 10, can_socket,
    )
    left_motor_aft = HubMotor(
        'left_motor_aft', 11, can_socket,
    )
    motors = [right_motor, right_motor_aft, left_motor, left_motor_aft]

    if True:
        print('reboot')
        for motor in motors:
            motor.reboot()

        time.sleep(1)

        print('rebooted')

    left_motor.set_input_velocity(0)
    right_motor.set_input_velocity(0)

    if True:
        for motor in motors:
            motor.clear_errors()
        for motor in motors:
            #motor.set_requested_state(motor_pb2.ODriveAxis.STATE_IDLE)
            motor.set_requested_state(motor_pb2.ODriveAxis.STATE_CLOSED_LOOP_CONTROL)

    #left_motor_aft.set_requested_state(motor_pb2.ODriveAxis.STATE_IDLE)
    #right_motor_aft.set_requested_state(motor_pb2.ODriveAxis.STATE_IDLE)
    count = [0]

    global x
    x = 0

    def command_loop():
        global x
        periodic.read()

        if count[0] % (2*command_rate_hz) == 0:
            logger.info(
                '\nright: %s\nleft: %s\nright_aft: %s\nleft_aft: %s',
                MessageToString(right_motor.get_state(), as_one_line=True),
                MessageToString(left_motor.get_state(), as_one_line=True),
                MessageToString(right_motor_aft.get_state(), as_one_line=True),
                MessageToString(left_motor_aft.get_state(), as_one_line=True),
            )
        x += 1.0/command_rate_hz

        right_motor.set_input_velocity(0)
        left_motor.set_input_velocity(0)
        count[0] += 1

    loop.add_reader(can_socket, lambda: can_socket.recv())
    loop.add_reader(periodic, command_loop)
    loop.run_forever()


if __name__ == '__main__':
    main()
