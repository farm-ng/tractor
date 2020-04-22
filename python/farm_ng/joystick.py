# Modifications: by Ethan Rublee <ethan.rublee@gmail.com>
# Changes Licensed under: Apache 2.0 license
# Changes list:
#   - 04/02/2020 - ethan.rublee@gmail.com, Document source and new change licensed under apache 2.0
#   -              Make a Joystick class and make this file importable
#
# Original source was released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt
# Copied from https://gist.githubusercontent.com/rdb/8864666/raw/516178252bbe1cfe8067145b11223ee54c5d9698/js_linux.py
#
from fcntl import ioctl
import linuxfd
import logging
import os, struct, array
import select


logger = logging.getLogger("jk")
logger.setLevel(logging.INFO)


# These constants were borrowed from linux/input.h
axis_names = {
    0x00: "x",
    0x01: "y",
    0x02: "z",
    0x03: "rx",
    0x04: "ry",
    0x05: "rz",
    0x06: "trottle",
    0x07: "rudder",
    0x08: "wheel",
    0x09: "gas",
    0x0A: "brake",
    0x10: "hat0x",
    0x11: "hat0y",
    0x12: "hat1x",
    0x13: "hat1y",
    0x14: "hat2x",
    0x15: "hat2y",
    0x16: "hat3x",
    0x17: "hat3y",
    0x18: "pressure",
    0x19: "distance",
    0x1A: "tilt_x",
    0x1B: "tilt_y",
    0x1C: "tool_width",
    0x20: "volume",
    0x28: "misc",
}

button_names = {
    0x120: "trigger",
    0x121: "thumb",
    0x122: "thumb2",
    0x123: "top",
    0x124: "top2",
    0x125: "pinkie",
    0x126: "base",
    0x127: "base2",
    0x128: "base3",
    0x129: "base4",
    0x12A: "base5",
    0x12B: "base6",
    0x12F: "dead",
    0x130: "a",
    0x131: "b",
    0x132: "c",
    0x133: "x",
    0x134: "y",
    0x135: "z",
    0x136: "tl",
    0x137: "tr",
    0x138: "tl2",
    0x139: "tr2",
    0x13A: "select",
    0x13B: "start",
    0x13C: "mode",
    0x13D: "thumbl",
    0x13E: "thumbr",
    0x220: "dpad_up",
    0x221: "dpad_down",
    0x222: "dpad_left",
    0x223: "dpad_right",
    # XBox 360 controller uses these codes.
    0x2C0: "dpad_left",
    0x2C1: "dpad_right",
    0x2C2: "dpad_up",
    0x2C3: "dpad_down",
}


class Joystick(object):
    def __init__(self, device):
        self.axis_map = []
        self.button_map = []
        self.axis_states = {}
        self.button_states = {}

        # Open the joystick device.
        fn = device
        print("Opening %s..." % fn)
        self.jsdev = open(fn, "rb")

        # Get the device name.
        # buf = bytearray(63)
        buf = array.array("B", [0] * 64)
        ioctl(self.jsdev, 0x80006A13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b"\x00").decode("utf-8")
        print("Device name: %s" % js_name)

        # Get number of axes and buttons.
        buf = array.array("B", [0])
        ioctl(self.jsdev, 0x80016A11, buf)  # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array("B", [0])
        ioctl(self.jsdev, 0x80016A12, buf)  # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array("B", [0] * 0x40)
        ioctl(self.jsdev, 0x80406A32, buf)  # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = axis_names.get(axis, "unknown(0x%02x)" % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array("H", [0] * 200)
        ioctl(self.jsdev, 0x80406A34, buf)  # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = button_names.get(btn, "unknown(0x%03x)" % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        print("%d axes found: %s" % (num_axes, ", ".join(self.axis_map)))
        print("%d buttons found: %s" % (num_buttons, ", ".join(self.button_map)))

    def fileno(self):
        return self.jsdev.fileno()

    def read_event(self):
        evbuf = self.jsdev.read(8)
        if not evbuf:
            return None

        time, value, type, number = struct.unpack("IhBB", evbuf)

        if type & 0x80:
            print("(initial)", end="")

        if type & 0x01:
            button = self.button_map[number]
            if button:
                self.button_states[button] = value
                if value:
                    # print("%s pressed" % (button))
                    pass
                else:
                    # print("%s released" % (button))
                    pass

        if type & 0x02:
            axis = self.axis_map[number]
            if axis:
                fvalue = value / 32767.0
                self.axis_states[axis] = fvalue
                # print("%s: %.3f" % (axis, fvalue))


class MaybeJoystick:
    def __init__(self, device):
        self.device = device
        self.joystick = None
        self.timer = linuxfd.timerfd(rtc=False, nonBlocking=True)
        self.timer.settime(value=1.0, interval=1.0)
        self.open_joystick()

    def open_joystick(self):
        try:
            self.joystick = Joystick(self.device)
        except FileNotFoundError as e:
            logger.warning("Couldn't open joystick: %s" % str(e))
            self.joystick = None

    def get_axis_state(self, axis, default):
        if self.joystick is None:
            return default
        return self.joystick.axis_states[axis]

    def fileno(self):
        if self.joystick is None:
            return self.timer.fileno()
        else:
            return self.joystick.fileno()

    def read_event(self):
        if self.joystick is None:
            logger.warning("Joystick is none, timer count %s" % (self.timer.read()))
            self.open_joystick()
        else:
            try:
                self.joystick.read_event()
            except OSError as e:
                logger.warning("Error reading joystick: %s" % (str(e)))
                self.joystick = None


if __name__ == "__main__":
    js = Joystick()

    # Main event loop
    while True:
        rlist, wlist, xlist = select.select([js], [], [])

        if js in rlist:
            js.read_event()
