import asyncio
import logging
import math
import struct
import sys

import linuxfd
import numpy as np
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp

from farm_ng.core.ipc import get_event_bus
from farm_ng.core.ipc import make_event
#from farm_ng.tractor import motor_pb2
from farm_ng.motors.canbus import CANSocket

#from farm_ng.tractor.config import TractorConfigManager

logger = logging.getLogger('farm_ng.motor')

logger.setLevel(logging.INFO)


VESC_SET_DUTY = 0
VESC_SET_CURRENT = 1
VESC_SET_CURRENT_BRAKE = 2
VESC_SET_RPM = 3
VESC_SET_POS = 4
VESC_STATUS_MSG_1 = 9
VESC_STATUS_MSG_2 = 14
VESC_STATUS_MSG_3 = 15
VESC_STATUS_MSG_4 = 16
VESC_STATUS_MSG_5 = 27


def vesc_parse_status_msg_1(state, data):
    msg = struct.unpack(
        '>ihh',  # big endian, int32, int16, int16
        data,
    )
    rpm, current, duty_cycle = msg
    current /= 1e1
    duty_cycle /= 1e3
    logger.debug('rpm %d current %f duty_cycle %f', rpm, current, duty_cycle)
    state.rpm.value = rpm
    state.current.value = current
    state.duty_cycle.value = duty_cycle
    return state


def vesc_parse_status_msg_2(state, data):
    msg = struct.unpack(
        '>ii',  # big endian, int32, int32
        data,
    )
    amp_hours, amp_hours_charged = msg
    amp_hours = amp_hours/1e4
    amp_hours_charged = amp_hours_charged/1e4
    logger.debug(
        'amp_hours %f amp_hours_charged %f',
        amp_hours, amp_hours_charged,
    )
    state.amp_hours.value = amp_hours
    state.amp_hours_charged.value = amp_hours_charged
    return state


def vesc_parse_status_msg_3(state, data):
    msg = struct.unpack(
        '>ii',  # big endian, int32, int32
        data,
    )
    watt_hours, watt_hours_charged = msg
    watt_hours = watt_hours/1e4
    watt_hours_charged = watt_hours_charged/1e4
    logger.debug(
        'watt_hours %f watt_hours_charged %f',
        watt_hours, watt_hours_charged,
    )
    state.watt_hours.value = watt_hours
    state.watt_hours_charged.value = watt_hours_charged
    return state


def vesc_parse_status_msg_4(state, data):
    msg = struct.unpack(
        '>hhhh',  # big endian, int16, int16, int 16
        data,
    )
    temp_fet, temp_motor, current_in, pid_pos = msg
    temp_fet /= 1e1
    temp_motor /= 1e1
    current_in /= 1e1
    pid_pos /= 50.0

    logger.debug(
        'temp_fet %f temp_motor %f current_in %f pid_pos %f',
        temp_fet, temp_motor, current_in, pid_pos,
    )
    state.temp_fet.value = temp_fet
    state.temp_motor.value = temp_motor
    state.current_in.value = current_in
    state.pid_pos.value = pid_pos
    return state


def vesc_parse_status_msg_5(state, data):
    msg = struct.unpack(
        '>ihh',  # big endian, int32, int16, int16
        data,
    )
    tachometer, input_voltage, _ = msg
    input_voltage /= 1e1
    logger.debug('tachometer %f input_voltage %f', tachometer, input_voltage)
    # divide by 6 here to get in the same units as the RPM.
    # according to docs https://github.com/vedderb/bldc/blob/master/mcpwm_foc.c
    # RPM must be divided by 0.5*number of poles
    # and tachometer must be divided by 3*number of poles
    # rpm = erpm/(0.5*n_poles)
    # rotations = tach/(3*n_poles)
    # erpm = tach*(0.5*n_poles)/(3.0*n_poles)

    state.tachometer.value = tachometer / (6.0)  # get it in eRPM units
    state.input_voltage.value = input_voltage
    return state


g_vesc_msg_parsers = {
    VESC_STATUS_MSG_1: vesc_parse_status_msg_1,
    VESC_STATUS_MSG_2: vesc_parse_status_msg_2,
    VESC_STATUS_MSG_3: vesc_parse_status_msg_3,
    VESC_STATUS_MSG_4: vesc_parse_status_msg_4,
    VESC_STATUS_MSG_5: vesc_parse_status_msg_5,
}


class HubMotor:
    def __init__(
            self, name, wheel_radius, gear_ratio, poll_pairs,
            can_node_id, can_socket,
    ):
        self.name = name
        self.can_node_id = can_node_id
        self.can_socket = can_socket
        self.wheel_radius = wheel_radius
        self.gear_ratio = gear_ratio
        self.poll_pairs = poll_pairs
        self.max_current = 20
        self._event_bus = get_event_bus(self.name)
        self._latest_state = motor_pb2.MotorControllerState()
        self._latest_stamp = Timestamp()
        self.can_socket.add_reader(self._handle_can_message)
        self._last_tachometer_stamp = None
        self._delta_time_seconds = 0.0
        self._average_delta_time = 0.0

    def _handle_can_message(self, cob_id, data, stamp):
        can_node_id = (cob_id & 0xff)
        if can_node_id != self.can_node_id:
            return
        command = (cob_id >> 8) & 0xff
        parser = g_vesc_msg_parsers.get(command, None)
        if parser is None:
            logger.warning(
                'No parser for command :%x node id: %x', command, can_node_id,
            )
            return
        logger.debug('can node id %02x', can_node_id)
        parser(self._latest_state, data)
        self._latest_stamp.CopyFrom(stamp)

        if command == VESC_STATUS_MSG_5:
            if self._last_tachometer_stamp is not None:
                self._delta_time_seconds = (self._latest_stamp.ToMicroseconds() - self._last_tachometer_stamp.ToMicroseconds())*1e-6
                self._average_delta_time = self._average_delta_time*(0.9) + self._delta_time_seconds * 0.1
            else:
                self._last_tachometer_stamp = Timestamp()

            self._last_tachometer_stamp.CopyFrom(self._latest_stamp)

            # only log on the 5th vesc message, as we have complete state at that point.
            event = make_event('%s/state' % self.name, self._latest_state, stamp=self._latest_stamp)
            self._event_bus.send(event)

    def _send_can_command(self, command, data):
        cob_id = int(self.can_node_id) | (command << 8)
        # print('send %x'%cob_id, '%x'%socket.CAN_EFF_FLAG)
        # socket.CAN_EFF_FLAG for some reason on raspberry pi this is
        # the wrong value (-0x80000000 )
        eff_flag = 0x80000000
        self.can_socket.send(cob_id, data, flags=eff_flag)

    def _tach_to_rads(self, er):
        '''compute radians from electric revs'''
        rotations = er/(self.poll_pairs*self.gear_ratio)
        return rotations*2*math.pi

    def average_update_rate(self):
        return 1.0/max(self._average_delta_time, 1e-6)

    def tachometer_rads(self):
        return self._tach_to_rads(self._latest_state.tachometer.value)

    def velocity_rads(self):
        return self._tach_to_rads(self._latest_state.rpm.value)/60.0

    def send_rpm_command(self, rpm):
        RPM_FORMAT = '>i'  # big endian, int32
        erpm = rpm * self.poll_pairs*self.gear_ratio
        self._latest_state.commanded_rpm.value = int(erpm)
        self._latest_state.ClearField('commanded_brake_current')
        data = struct.pack(RPM_FORMAT, int(erpm))
        self._send_can_command(VESC_SET_RPM, data)

    def send_velocity_command(self, velocity_m_s):
        rpm = 60*velocity_m_s/(self.wheel_radius * 2*math.pi)
        self.send_rpm_command(rpm)

    def send_velocity_command_rads(self, rads_second):
        rpm = 60*rads_second/(2*math.pi)
        self.send_rpm_command(rpm)

    def send_current_command(self, current_amps):
        CURRENT_FORMAT = '>i'  # big endian, int32
        data = struct.pack(
            CURRENT_FORMAT, int(
                1000*max(
                    min(current_amps, self.max_current), -
                    self.max_current,
                ),
            ),
        )
        self._send_can_command(VESC_SET_CURRENT, data)

    def send_current_brake_command(self, current_amps):
        CURRENT_BRAKE_FORMAT = '>i'  # big endian, int32

        self._latest_state.ClearField('commanded_rpm')
        self._latest_state.commanded_brake_current.value = current_amps

        data = struct.pack(
            CURRENT_BRAKE_FORMAT, int(
                1000*np.clip(current_amps, 0, self.max_current),
            ),
        )
        self._send_can_command(VESC_SET_CURRENT_BRAKE, data)

    def get_state(self):
        return self._latest_state


def main():

    command_rate_hz = 100
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

    config = TractorConfigManager.saved()
    right_motor = HubMotor(
        'right_motor',
        config.wheel_radius.value,
        config.hub_motor_gear_ratio.value,
        config.hub_motor_poll_pairs.value, 20, can_socket,
    )
    right_motor_aft = HubMotor(
        'right_motor_aft',
        config.wheel_radius.value,
        config.hub_motor_gear_ratio.value,
        config.hub_motor_poll_pairs.value, 21, can_socket,
    )
    left_motor = HubMotor(
        'left_motor',
        config.wheel_radius.value,
        config.hub_motor_gear_ratio.value,
        config.hub_motor_poll_pairs.value, 10, can_socket,
    )
    left_motor_aft = HubMotor(
        'left_motor_aft',
        config.wheel_radius.value,
        config.hub_motor_gear_ratio.value,
        config.hub_motor_poll_pairs.value, 11, can_socket,
    )

    count = [0]

    def command_loop():
        periodic.read()
        if count[0] % (2*command_rate_hz) == 0:
            logger.info(
                'right: %s\nleft: %s',
                MessageToString(right_motor.get_state(), as_one_line=True),
                MessageToString(left_motor.get_state(), as_one_line=True),
            )
        right_motor.send_velocity_command(0.0)
        # right_motor_aft.send_velocity_command(0.0)
        left_motor.send_velocity_command(0.0)
        # left_motor_aft.send_velocity_command(0.0)
        count[0] += 1

    loop.add_reader(can_socket, lambda: can_socket.recv())
    loop.add_reader(periodic, command_loop)
    loop.run_forever()


if __name__ == '__main__':
    main()
