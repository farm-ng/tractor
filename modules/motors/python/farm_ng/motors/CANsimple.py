#!/usr/bin/env python3


import argparse
import atexit
import math
import struct
import sys
import time
from collections import OrderedDict
from threading import Thread

import can  # needs python3-can debian package or equivalent
from enums import ODenum
from enums import ODerror
from sortedcontainers import \
    SortedDict  # needs python3-sortedcontainers debian package or equivalent - this is needed to access list of nodes by ID as well as by index (which is needed for QT based GUI)

# ** odrive_cansimple_cmd()
# generic CAN command function for "CAN Simple" protocol
# see: https://github.com/madcowswe/ODrive/blob/RazorsFrozen/docs/can-protocol.md
# @param node_id: ODrive assigned CAN bus node ID, one per axis
# @param cmd_id: Command ID as per messages table
# @param data: Python list of signal values, according to messages table
# @param format: Python Struct.pack format string for signals in 8-byte CAN message, encoded according to signals table
# @param RTR: Remote Transmission Request bit - set to request "Get" data from ODrive


def odrive_cansimple_cmd(bus, node_id, cmd_id, data=[], format='', RTR=False):
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((node_id << 5) | cmd_id), data=data_frame)
    msg.is_remote_frame = RTR
    msg.is_extended_id = False
    bus.send(msg)

# ** odrive_cansimple_receive(can.msg)
# Pass in a CAN message to be decoded into CANsimple node ID, command ID and data packet


def odrive_cansimple_receive(msg):
    if(msg.is_extended_id):
        # we don't support extended IDs in CANsimple, so discard these messages as not relevant
        raise ValueError('Ignoring extended-ID: %d' % msg.arbitration_id)
    if(msg.is_remote_frame):
        raise ValueError('Ignoring remote frame request: %d' % msg.arbitration_id)
    node_id = (msg.arbitration_id >> 5) & 0b11111   # upper 6 bits
    command_id = (msg.arbitration_id & 0b11111)     # lower 5 bits
    return (node_id, command_id, msg.data)


# TODO: Split CANSimple into its own file and support other protocols over CAN

# ODrive_Proto_CANsimple
# Defines the "CANsimple" protocol messages
class ODrive_Proto_CANsimple:
    def __init__(self):
        self.cmd = dict()
        self.init_cmds()
        self.signals = list()

    # ** initialise commands table
    # see: https://docs.odriverobotics.com/can-protocol
    # New commands should be added to this list as they are defined
    def init_cmds(self):
        self.add_cmd(ODCmd(0x000, 'NMT'))
        self.add_cmd(
            ODCmd(
                0x001, 'Heartbeat',          [
                    Sig('axis.error', 'uint32', enum=ODerror.axis_error),
                    Sig('axis.state', 'uint32', enum=ODenum.axis_state),
                ],
            ),
        )
        self.add_cmd(ODCmd(0x002, 'STOP'))
        self.add_cmd(ODCmd(0x003, 'GetMotorError',      [Sig('motor.error', 'uint32', enum=ODerror.motor_error)],  RTR=True))
        self.add_cmd(ODCmd(0x004, 'GetEncoderError',    [Sig('encoder.error', 'uint32', enum=ODerror.encoder_error)],  RTR=True))
        self.add_cmd(ODCmd(0x005, 'GetSensorlessError', [Sig('sensorless.error', 'uint32', enum=ODerror.sensorless_error)],  RTR=True))
        self.add_cmd(ODCmd(0x006, 'SetNodeID',          [Sig('axis.config.nodeID', 'uint16')]))
        self.add_cmd(ODCmd(0x007, 'SetRequestedState',  [Sig('axis.requestedState', 'uint32', enum=ODenum.axis_state)]))
        self.add_cmd(ODCmd(0x008, 'SetStartupConfig',   [Sig('axis.config.startup', 'uint32')]))
        self.add_cmd(ODCmd(0x009, 'GetEncoderEstimate', [Sig('encoder.pos_estimate', 'float'),         Sig('encoder.vel_estimate', 'float')],    RTR=True))
        self.add_cmd(ODCmd(0x00A, 'GetEncoderCount',    [Sig('encoder.shadow_count', 'int32'),         Sig('encoder.count_in_cpr', 'int32')],    RTR=True))
        self.add_cmd(
            ODCmd(
                0x00B, 'SetControllerModes', [
                    Sig(
                        'controller.config.control_mode', 'uint32',
                        enum=ODenum.control_mode,
                    ),         Sig('controller.config.input_mode', 'uint32', enum=ODenum.input_mode),
                ],
            ),
        )
        self.add_cmd(
            ODCmd(
                0x00C, 'SetInputPos',        [
                    Sig('controller.input_pos', 'int32'),                    Sig(
                    'controller.vel_ff', 'int16', 0,  0.1,
                    ),     Sig('controller.cur_ff', 'int16', 0, 0.01),
                ],
            ),
        )
        self.add_cmd(ODCmd(0x00D, 'SetInputVel',        [Sig('controller.input_vel', 'float', 0, 1), Sig('controller.cur_ff', 'float', 0, 1)]))
        self.add_cmd(ODCmd(0x00E, 'SetInputCurrent',    [Sig('controller.input_current', 'int32', 0.01)]))
        self.add_cmd(ODCmd(0x00F, 'SetVelocityLimit',   [Sig('controller.config.vel_limit', 'float')]))
        self.add_cmd(ODCmd(0x010, 'StartAnticogging'))
        self.add_cmd(ODCmd(0x011, 'SetTrajVelLimit',    [Sig('axis.config.traj_vel_limit', 'float')]))
        self.add_cmd(ODCmd(0x012, 'SetTrajAccelLimits', [Sig('axis.config.traj_accel_limit', 'float'),      Sig('axis.config.traj_decel_limit', 'float')]))
        self.add_cmd(ODCmd(0x013, 'SetTrajApcs',        [Sig('axis.config.traj_apcs', 'float')]))
        self.add_cmd(ODCmd(0x014, 'GetIq',              [Sig('motor.iq_setpoint', 'float'),            Sig('motor.iq_measured', 'float')],       RTR=True))
        self.add_cmd(ODCmd(0x015, 'GetSensorlessEstimates', [Sig('sensorless.pos_estimate', 'float'),  Sig('sensorless.vel_estimate', 'float')], RTR=True))
        self.add_cmd(ODCmd(0x016, 'Reboot'))
        self.add_cmd(ODCmd(0x017, 'GetVBus',            [Sig('axis.vbus', 'float')],  RTR=True))
        self.add_cmd(ODCmd(0x018, 'ClearErrors'))
        # not yet implemented in firmware
        self.add_cmd(ODCmd(0x019, 'GetSerialNo',            [Sig('axis.serialNo', 'uint32')],  RTR=True))
        self.add_cmd(ODCmd(0x700, 'CANOpenHeartbeat'))

    def add_cmd(self, cmd):
        self.cmd[cmd.id] = cmd
        self.cmd[cmd.name] = cmd
        self.cmd[str(cmd.id)] = cmd
     #   for sig in cmd.signals:
     #       self.signals.append(sig.name)

    def get_cmds(self):
        result = dict()
        for key in self.cmd:
            if(key == self.cmd[key].name):
                result[key] = self.cmd[key]
                #print("%d: %s" % (self.cmd[key].id, self.cmd[key].name))
        return result

    def get_dictionary_items(self):
        cmds = self.get_cmds()
        result = list()
        for key in cmds:
            list.append(self.cmd[key].signals)
        return sorted(result)


class ODCmd:
    def __init__(self, cmd_id, name, signals=[], RTR=False):
        self.id = int(cmd_id)
        self.name = str(name)
        self.signals = OrderedDict()
        self.rtr = bool(RTR)
        for (n, sig) in enumerate(signals):
            self.signals[sig.name] = sig
        self.format = self.data_format()

    # coding of various types according to struct.pack
    # https://docs.python.org/3/library/struct.html
    data_coding = dict({
        'uint16': 'H',
        'int16': 'h',
        'uint32': 'L',
        'int32': 'l',
        'uint64': 'Q',
        'int64': 'q',
        'float': 'f',
        'enum': 'L',
        'error': 'L',
    })

    def data_format(self):
        fmt = '<'
        for sig in self.signals.values():
            # print(sig.format)
            fmt = fmt + self.data_coding[sig.format]
        return fmt


class Sig:
    def __init__(self, name, data_format, default=None, scale=1, enum=None):
        self.name = name
        self.format = data_format
        self.scale = scale
        self.default = default
        self.enum = enum

    def parse(self, arg):
        if(self.format == 'float'):
            return float(arg)
        else:
            return int(arg)


# ** ODriveCANBus
# CAN I/O thread - listens for messages, populates a data model based on sent and recieved data

class ODriveCANBus(Thread):
    def __init__(self, bus, proto=None, model=None):
        Thread.__init__(self)
        self.bus = bus
        if(proto is None):
            self.proto = ODrive_Proto_CANsimple()
        else:
            self.proto = proto
        if(model is None):
            self.node_list = ODriveBusModel()
        else:
            self.node_list = model
        self.probe_node = 0
        self.stopped = False
        self.messages_received = 0
        self.messages_sent = 0

    def run(self):
        for msg in self.bus:
            try:
                self.messages_received = self.messages_received + 1
                if(self.stopped):
                    break
                (node_id, cmd_id, data) = odrive_cansimple_receive(msg)
                if node_id in self.node_list:
                    self.receive_message(node_id, cmd_id, data)
                elif cmd_id == self.proto.cmd['Heartbeat'].id:  # add new node
                    print('Found node id %d' % node_id)
                #    self.probe_node = node_id
                #    self.send_command('GetSerialNo');  # TODO: Add protocol command to get serial no
                #       For now just assume any heatrbeat message cam from a bona fide ODrive
                # elif cmd_id == proto.cmd['GetSerialNo'].id and node_id == self.probe_node:
                    self.node_list[node_id] = ODriveAxisModel(node_id, self.proto)
                else:
                    print('Unknown message: NodeID=%d, CMD=0x%x, data=%s' % (node_id, cmd_id, data))
                try:
                    self.node_list[node_id].updateViews()
                except KeyError as e:
                    print('Unknown Node %s' % e)
            except ValueError as e:
                # print(e)
                pass

    def stop(self):
        self.stopped = True

    def send_message(self, node_id, cmd_id_or_name, *params):
        # lookup command in list
        cmd = self.proto.cmd[cmd_id_or_name]

        # store data in dictionary
        for (n, value) in enumerate(params):
            sig_name = list(cmd.signals.values())[n].name  # FIXME: nasty re-writing as list each time
            if(node_id in self.node_list):
                self.node_list[node_id][sig_name] = value
            else:
                print('Warning: Not Sending command to unknown node ID: %d' % node_id)
                return

        # send command
        #data = [sig.value for sig in cmd.signals]
        data = list()
        for(n, sig) in enumerate(cmd.signals.values()):
            if(n >= len(params)):
                data.append(sig.default)
            else:
                data.append(sig.parse(params[n]))

        if(cmd.rtr):
            odrive_cansimple_cmd(self.bus, node_id, cmd.id, RTR=True)  # no data payload for remote frame request
        else:
            odrive_cansimple_cmd(self.bus, node_id, cmd.id, data, cmd.format, RTR=False)
#        print("Sent CANsimple message: %s" % cmd.name)
        self.messages_sent = self.messages_sent + 1

    def receive_message(self, node_id, cmd_id, data):
        cmd = self.proto.cmd[cmd_id]
#        print("Recieved CANsimple message: %s" % cmd.name)
        if(len(cmd.signals) == 0):
            return

        # oDrive always transmits 8 bytes even when there is less tyhan 8 bytes to send
        # this confuses struct.unpack, so truncate the bytearray to the right size
        data = data[0:struct.calcsize(cmd.format)]

        values = struct.unpack(cmd.format, data)
        for (n, value) in enumerate(values):
            sig_name = list(cmd.signals.values())[n].name  # FIXME: nasty
            sig = cmd.signals[sig_name]
            # apply scaling
            if(sig.scale != 1):
                value = value * sig.scale
            if(sig.enum is not None):
                try:
                    value_str = sig.enum[value]
                except KeyError:
                    value_str = str(ODerror.get_errors(sig.enum, value))
                #self.data_model["axis%d.%s_str" % (node_id, sig_name)] = value_str
                self.node_list[node_id][sig_name + '_str'] = value_str
            self.node_list[node_id][sig_name] = value
            #self.data_model["axis%d.%s" % (node_id, sig_name)] = value
#            print(self.node_list[node_id].DataModel)


# TODO: Split data model into separate file
# TODO: Have just one data dict, not N+1
# TODO: Support multiple protocols

class ODriveDataModel(SortedDict):
    def __init__(self, proto=None):
        super().__init__()
        self.updateFunc = None
        if(proto is None):
            self.proto = ODrive_Proto_CANsimple()
        else:
            self.proto = proto
    # returns the nth device, sorted by ID

    def elementAt(self, index):
        try:
            return self.keys().__getitem__(index)
        except (KeyError, IndexError) as e:
            return -1
     #   skv = self.keys()
      #  s
     #   print(index)
     #   print(skv)

     #   if index in skv:
      #      key = skv[index]
      #      return key
      #  else:
      #      return -1
    # set function to be called when data changes
    def setUpdateFunc(self, func):
        self.updateFunc = func
    # override __setitem__ to know when data has changed

    def __setitem__(self, key, value):
        ret = super().__setitem__(key, value)
        self.updateViews()

    def updateViews(self):
        if(self.updateFunc is not None):
            self.updateFunc()

    # find row number of a given item
    def indexOf(self, key):
        return self.keys().index(key)


class ODriveBusModel(ODriveDataModel):
    def __init__(self, test=False, proto=None):
        super().__init__(proto)
        if(test):
            self[66] = ODriveAxisModel(66, proto)


class ODriveAxisModel(ODriveDataModel):
    def __init__(self, node_id, proto=None):
        super().__init__(proto)
        self.node_id = node_id
        self['axis.config.node_id'] = node_id


# ** UpdaterThread
# Generates messages to query data

class UpdaterThread(Thread):
    def __init__(self, bus, model, cmd_rate_delay=0.01, proto=None):
        super().__init__()
        self.bus = bus
        self.model = bus.node_list
        self.cmd_delay_s = cmd_rate_delay
        self.running = True
        if(proto is None):
            self.proto = ODrive_Proto_CANsimple()
        else:
            self.proto = proto
        self.cmdlist = ['GetEncoderCount', 'GetEncoderEstimate', 'GetVBus', 'GetIq']
        self.cmdlist_error = ['GetMotorError', 'GetEncoderError']

    def run(self):
        while(self.running is True):
            for axis in self.model:
                for cmd in self.cmdlist:
                    self.bus.send_message(axis, cmd)
                    time.sleep(self.cmd_delay_s)
                # print(self.model[axis])

    def stop(self):
        self.running = False


# TODO: test-code has got pretty nasty - CLEAN ME
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        'CANSimple.py',
        description='CAN interface driver for ODrive. This file can be run on its own to test the interface',
    )
    # add_help=False)

    parser.add_argument('Command', help="Command Name, or 'Help' to list supported commands", default='GetVBus')
    parser.add_argument('Args', help='command arguments', nargs='*')
    parser.add_argument('-n', '--node_id', type=int, help='Axis Node ID', default=1, dest='node_id')
    parser.add_argument('-B', '--bustype', help='Bus type', default='socketcan', dest='bustype')
    parser.add_argument('-i', '--interface', help='CAN interface channel name', default='can0', dest='channel')
    #parser.add_argument("-L", "--list-cmds", action="store_true", help="List supported commands", default=False, dest="list_cmds")
    parser.add_argument('-R', '--repeat-num', type=float, help='Repeat R times, -1 = inf', default=1, dest='repeat_n')
    parser.add_argument('-r', '--repeat-period', type=float, help='Repeat every r seconds', default=1, dest='repeat_f')
    #parser.add_argument("-h", "--help", help="This help text", action="store_true")
    parser.add_argument('-U', '--auto-update', help='Update data from all connected drives', default=False, dest='autoUpdate', action='store_true')

    parse_error = False
    try:
        args = parser.parse_args()
    except (argparse.ArgumentError, SystemExit) as e:
        print(e)
        parse_error = True

    proto = ODrive_Proto_CANsimple()
    cmdlist = proto.get_cmds()
    if(parse_error is True or args.Command.upper() == 'HELP'):
        parser.print_help()
        print('=== CMD Number | CMD ID | [CMD Param ... ] ===')
        for cmd in cmdlist.values():
            if(cmd.rtr == False):
                params = [str(c.name).rsplit('.', 1)[1] for c in cmd.signals.values()]
            else:
                params = ''
            print('%d:\t%s %s' % (cmd.id, cmd.name, params))
        print("""
           EXAMPLES:
           CANsimple.py -i can0 -n 11 GetVBus -R 10
           CANsimple.py -i can0 -n 11 SetInputPos 1000
           CANsimple.py -i can0 -n 11 -U GetVBus -R 100 -r 0.1""")
        sys.exit(0)

    bus = can.interface.Bus(channel=args.channel, bustype=args.bustype)

    OD = ODriveCANBus(bus, proto)
    atexit.register(OD.stop)

    OD.start()
    time.sleep(1)

    UPD = UpdaterThread(OD, proto)
    if(args.autoUpdate):
        atexit.register(UPD.stop)
        UPD.start()

    try:
        n = 0
        while(True):
            n = n + 1
            time.sleep(args.repeat_f)
            OD.send_message(args.node_id, args.Command, *args.Args)
            if (n >= args.repeat_n and args.repeat_n != -1):
                break
            else:
                for node_id in OD.node_list:
                    print('=== ODrive Node ID 0x%d ===' % node_id)
                    print(OD.node_list[node_id])

    except KeyboardInterrupt as e:
        sys.exit(0)
    except KeyError as e:
        print("Unknown Command: %s. Try 'help'." % e)

    time.sleep(1)
    print('=== Communication Finished ===')
    print('Total messages sent: %d' % OD.messages_sent)
    print('Total messages recieved: %d' % OD.messages_received)
    print('Detected ODrive axes: %d' % len(OD.node_list))

    for node_id in OD.node_list:
        print('=== ODrive Node ID 0x%d ===' % node_id)
        print(OD.node_list[node_id])

    OD.stop()
    UPD.stop()
