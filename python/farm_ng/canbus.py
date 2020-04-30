#!/usr/bin/env python
# Modifications: by Ethan Rublee <ethan.rublee@gmail.com>
# Changes Licensed under: Apache 2.0 license
# Changes list:
#   - 04/02/2020 - ethan.rublee@gmail.com,
#                  Document source and new change licensed under apache 2.0
#                  add fileno() member function to support select
# original source (https://github.com/abencz/python_socketcan)
# commit ed6b7faa3023c2155959d2f83242f706fd44040f
#  (HEAD -> master, origin/master, origin/HEAD)
# Author: Alex Bencz <abencz@gmail.com>
# Date:   Mon Jul 11 13:19:02 2016 -0400
# Copyright (c) 2016 Alex Bencz
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import argparse
import errno
import socket
import struct
import sys
import asyncio


class CANSocket(object):
    FORMAT = '<IB3x8s'
    FD_FORMAT = '<IB3x64s'
    CAN_RAW_FD_FRAMES = 5

    def __init__(self, interface=None):
        self.sock = socket.socket(
            socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW,
        )
        if interface is not None:
            self.bind(interface)
        self.readers = []

    def add_reader(self, reader):
        self.readers.append(reader)
        
    def fileno(self):
        # for select.
        return self.sock.fileno()

    def bind(self, interface):
        self.sock.bind((interface,))
        # self.sock.setsockopt(socket.SOL_CAN_RAW, self.CAN_RAW_FD_FRAMES, 1)

    def send(self, cob_id, data, flags=0):
        cob_id = cob_id | flags
        can_pkt = struct.pack(self.FORMAT, cob_id, len(data), data)
        self.sock.send(can_pkt)

    def recv(self, flags=0):
        can_pkt = self.sock.recv(72)

        if len(can_pkt) == 16:
            cob_id, length, data = struct.unpack(self.FORMAT, can_pkt)
        else:
            cob_id, length, data = struct.unpack(self.FD_FORMAT, can_pkt)

        cob_id &= socket.CAN_EFF_MASK
        data = data[:length]
        for reader in self.readers:
            reader(cob_id, data)
        return cob_id, data



def format_data(data):
    return ''.join([hex(byte)[2:] for byte in data])


def generate_bytes(hex_string):
    if len(hex_string) % 2 != 0:
        hex_string = '0' + hex_string

    int_array = []
    for i in range(0, len(hex_string), 2):
        int_array.append(int(hex_string[i:i+2], 16))

    return bytes(int_array)


def send_cmd(args):
    try:
        s = CANSocket(args.interface)
    except OSError as e:
        sys.stderr.write(
            f'Could not send on interface {args.interface}\n',
        )
        sys.exit(e.errno)

    try:
        cob_id = int(args.cob_id, 16)
    except ValueError:
        sys.stderr.write(f'Invalid cob-id {args.cob_id}\n')
        sys.exit(errno.EINVAL)

    s.send(
        cob_id, generate_bytes(args.body),
        socket.CAN_EFF_FLAG if args.extended_id else 0,
    )


def listen_cmd(args):
    try:
        s = CANSocket(args.interface)
    except OSError as e:
        sys.stderr.write(
            f'Could not listen on interface {args.interface}\n',
        )
        sys.exit(e.errno)

    print(f'Listening on {args.interface}')
    loop = asyncio.get_event_loop()
    def read():
        cob_id, data = s.recv()
        print('{} {:05x}#{}'.format(args.interface, cob_id, format_data(data)))
    loop.add_reader(s, read)
    loop.run_forever()


def parse_args():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    send_parser = subparsers.add_parser('send', help='send a CAN packet')
    send_parser.add_argument(
        'interface', type=str,
        help='interface name (e.g. vcan0)',
    )
    send_parser.add_argument(
        'cob_id', type=str, help='hexadecimal COB-ID (e.g. 10a)',
    )
    send_parser.add_argument(
        'body', type=str, nargs='?', default='',
        help='hexadecimal msg body up to 8 bytes long (e.g. 00af0142fe)',
    )
    send_parser.add_argument(
        '-e', '--extended-id', action='store_true', default=False,
        help='use extended (29 bit) COB-ID',
    )
    send_parser.set_defaults(func=send_cmd)

    listen_parser = subparsers.add_parser(
        'listen', help='listen for and print CAN packets',
    )
    listen_parser.add_argument(
        'interface', type=str, help='interface name (e.g. vcan0)',
    )
    listen_parser.set_defaults(func=listen_cmd)

    return parser.parse_args()


def main():
    args = parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
