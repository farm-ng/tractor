import logging
import math
import select
import socket
import struct
import time

import linuxfd
from farm_ng.canbus import CANSocket
from farm_ng.joystick import MaybeJoystick

VESC_SET_DUTY = 0
VESC_SET_CURRENT = 1
VESC_SET_CURRENT_BRAKE = 2
VESC_SET_RPM = 3
VESC_SET_POS = 4


logger = logging.getLogger('dr')
logger.setLevel(logging.INFO)


def send_rpm_command(canbus, motor_id, rpm):
    RPM_FORMAT = '>i'  # big endian, int32
    data = struct.pack(RPM_FORMAT, int(rpm))
    cob_id = int(motor_id) | (VESC_SET_RPM << 8)
    # print('send %x'%cob_id)
    canbus.send(cob_id, data, flags=socket.CAN_EFF_FLAG)


def send_current_command(canbus, motor_id, current_amps):
    CURRENT_FORMAT = '>i'  # big endian, int32
    data = struct.pack(
        CURRENT_FORMAT, int(max(min(current_amps * 1000, 20000), -20000)),
    )
    cob_id = int(motor_id) | (VESC_SET_CURRENT << 8)
    # print('send %x'%cob_id)
    canbus.send(cob_id, data, flags=socket.CAN_EFF_FLAG)


def main_testfwd():
    canbus = CANSocket('can0')
    while True:
        send_rpm_command(canbus, 9, 3000)
        send_rpm_command(canbus, 7, 3000)
        time.sleep(0.02)


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


def main():
    command_rate_hz = 50
    command_period_seconds = 1.0 / command_rate_hz

    canbus = CANSocket('can0')
    joystick = MaybeJoystick('/dev/input/js0')

    # rtc=False means a monotonic clock for realtime loop as it won't
    # be adjusted by the system admin
    periodic = linuxfd.timerfd(rtc=False, nonBlocking=True)
    # here we start a timer that will fire in one second, and then
    # each command period after that
    periodic.settime(value=1.0, interval=command_period_seconds)

    # Main event loop
    fd_list = [periodic, canbus, joystick]

    while True:
        rlist, wlist, xlist = select.select(fd_list, [], [])

        if periodic in rlist:
            n_periods = periodic.read()
            if joystick.get_axis_state('brake', -1) == 1.0:
                speed = -joystick.get_axis_state('y', 0)
                turn = -joystick.get_axis_state('z', 0)
                left, right = steering(speed, turn)
                logger.debug(
                    'periodic %d speed %f left %f right %f',
                    n_periods, speed, left, right,
                )
                send_rpm_command(canbus, 7, 5000 * right)
                send_rpm_command(canbus, 9, 5000 * left)
            else:
                if joystick.is_connected():
                    send_rpm_command(canbus, 7, 0)
                    send_rpm_command(canbus, 9, 0)
                else:
                    send_current_command(canbus, 7, 0)
                    send_current_command(canbus, 9, 0)

        if joystick in rlist:
            joystick.read_event()
        if canbus in rlist:
            cob_id, data = canbus.recv()
            # print('%s %03x#%s' % ('can0', cob_id, format_data(data)))


if __name__ == '__main__':
    main()
