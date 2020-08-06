import asyncio
import logging
import math
import socket
import struct
import sys
import numpy as np
from farm_ng.ipc import get_event_bus, make_event
import linuxfd
from farm_ng.canbus import CANSocket
from farm_ng_proto.tractor.v1 import motor_pb2
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp

event_bus = get_event_bus()

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


def vesc_parse_status_msg_1(data):
    msg = struct.unpack(
        '>ihh',  # big endian, int32, int16, int16
        data,
    )
    rpm, current, duty_cycle = msg
    current /= 1e1
    duty_cycle /= 1e3
    logger.debug('rpm %d current %f duty_cycle %f', rpm, current, duty_cycle)
    state = motor_pb2.MotorControllerState()
    state.rpm.value = rpm
    state.current.value = current
    state.duty_cycle.value = duty_cycle
    return state


def vesc_parse_status_msg_2(data):
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
    state = motor_pb2.MotorControllerState()
    state.amp_hours.value = amp_hours
    state.amp_hours_charged.value = amp_hours_charged
    return state


def vesc_parse_status_msg_3(data):
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
    state = motor_pb2.MotorControllerState()
    state.watt_hours.value = watt_hours
    state.watt_hours_charged.value = watt_hours_charged
    return state


def vesc_parse_status_msg_4(data):
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
    state = motor_pb2.MotorControllerState()
    state.temp_fet.value = temp_fet
    state.temp_motor.value = temp_motor
    state.current_in.value = current_in
    state.pid_pos.value = pid_pos
    return state


def vesc_parse_status_msg_5(data):
    msg = struct.unpack(
        '>ihh',  # big endian, int32, int16, int16
        data,
    )
    tachometer, input_voltage, _ = msg
    input_voltage /= 1e1
    logger.debug('tachometer %f input_voltage %f', tachometer, input_voltage)
    state = motor_pb2.MotorControllerState()
    state.tachometer.value = tachometer
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
        self._latest_state = motor_pb2.MotorControllerState()
        self._latest_stamp = Timestamp()
        self.can_socket.add_reader(self._handle_can_message)

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
        state_msg = parser(data)
        self._latest_state.MergeFrom(state_msg)
        self._latest_stamp.CopyFrom(stamp)

        if command == VESC_STATUS_MSG_5:
            # only log on the 5th vesc message, as we have complete state at that point.
            event = make_event('%s/state' % self.name, self._latest_state, stamp=self._latest_stamp)
            event_bus.send(event)

    def _send_can_command(self, command, data):
        cob_id = int(self.can_node_id) | (command << 8)
        # print('send %x'%cob_id, '%x'%socket.CAN_EFF_FLAG)
        # socket.CAN_EFF_FLAG for some reason on raspberry pi this is
        # the wrong value (-0x80000000 )
        eff_flag=0x80000000 
        self.can_socket.send(cob_id, data, flags=eff_flag)

    def send_rpm_command(self, rpm):
        RPM_FORMAT = '>i'  # big endian, int32
        erpm = rpm * self.poll_pairs*self.gear_ratio
        data = struct.pack(RPM_FORMAT, int(erpm))
        self._send_can_command(VESC_SET_RPM, data)

    def send_velocity_command(self, velocity_m_s):
        rpm = 60*velocity_m_s/(self.wheel_radius * 2*math.pi)
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
        data = struct.pack(
            CURRENT_BRAKE_FORMAT, int(
                1000*np.clip(current_amps, 0, self.max_current))
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
            f'Could not listen on interface can0\n',
        )
        sys.exit(e.errno)

    print(f'Listening on can0')
    loop = asyncio.get_event_loop()

    radius = (17.0*0.0254)/2.0  # in meters, 17" diameter wheels
    gear_ratio = 29.9
    poll_pairs = 8
    right_motor = HubMotor('right_motor', radius, gear_ratio, poll_pairs, 7, can_socket)
    left_motor = HubMotor('left_motor', radius, gear_ratio, poll_pairs, 9, can_socket)

    count = [0]

    def command_loop():
        periodic.read()
        if count[0] % (30*command_rate_hz) == 0:
            plog.writer().flush()
        if count[0] % (2*command_rate_hz) == 0:
            logger.info(
                'right: %s\nleft: %s',
                MessageToString(right_motor.get_state(), as_one_line=True),
                MessageToString(left_motor.get_state(), as_one_line=True),
            )
        right_motor.send_velocity_command(0.0)
        left_motor.send_velocity_command(0.0)
        count[0] += 1

    loop.add_reader(can_socket, lambda: can_socket.recv())
    loop.add_reader(periodic, command_loop)
    loop.run_forever()


if __name__ == '__main__':
    main()
