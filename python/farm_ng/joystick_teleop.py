from farm_ng.canbus import CANSocket, format_data
import socket
from farm_ng.joystick import Joystick
import sys
import select
import linuxfd
import time
import math
import struct

VESC_SET_DUTY=0
VESC_SET_CURRENT=1
VESC_SET_CURRENT_BRAKE=2
VESC_SET_RPM=3
VESC_SET_POS=4
        
def send_rpm_command(canbus, motor_id, rpm):
    RPM_FORMAT='>i' #big endian, int32
    data=struct.pack(RPM_FORMAT, int(rpm))
    cob_id = int(motor_id) |(VESC_SET_RPM << 8)
    #print('send %x'%cob_id)
    canbus.send(cob_id, data, flags=socket.CAN_EFF_FLAG)

def main_testfwd():
    canbus = CANSocket('can0')
    while True:
        send_rpm_command(canbus, 9, 3000)
        send_rpm_command(canbus, 7, 3000)
        time.sleep(0.02)
 
def steering(x, y):
    #https://electronics.stackexchange.com/a/293108
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
    command_rate_hz=50
    command_period_seconds=1.0/command_rate_hz
    
    canbus = CANSocket('can0')
    joystick = Joystick()

    #rtc=False means a monotonic clock for realtime loop as it won't be adjusted by the system admin
    periodic = linuxfd.timerfd(rtc=False, nonBlocking=True) 
    # here we start a timer that will fire in one second, and then each command period after that
    periodic.settime(value=1.0, interval=command_period_seconds)

    
    # Main event loop
    
    while True:
        rlist,wlist,xlist=select.select([periodic, canbus, joystick], [], [])
        if periodic in rlist:
            n_periods =periodic.read()
            speed = -joystick.axis_states['y']
            turn = -joystick.axis_states['z']
            left, right = steering(speed,turn)
            #print('periodic %d speed %f left %f right %f'%(n_periods, speed, left, right))
            send_rpm_command(canbus, 7, 13000*right)
            send_rpm_command(canbus, 9, 13000*left)
        if joystick in rlist:
            joystick.read_event()
        if canbus in rlist:
            cob_id, data = canbus.recv()
            #print('%s %03x#%s' % ('can0', cob_id, format_data(data)))


if __name__ == "__main__":
    try:
        main()
    except OSError as e:
      print(e)
      sys.exit(e.errno)

