# socket_multicast_receiver.py
import asyncio
import logging
import socket
import struct
import sys
import time

import farm_ng_proto.tractor.v1.io_pb2
import farm_ng_proto.tractor.v1.motor_pb2
import farm_ng_proto.tractor.v1.steering_pb2
import farm_ng_proto.tractor.v1.tracking_camera_pb2
from farm_ng.periodic import Periodic
from farm_ng_proto.tractor.v1.io_pb2 import Announce
from farm_ng_proto.tractor.v1.io_pb2 import Event
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp


logger = logging.getLogger('ipc')
logger.setLevel(logging.INFO)


# https://en.wikipedia.org/wiki/Multicast_address
# adminstratively scoped: 239.0.0.0 to 239.255.255.255
_g_multicast_group = ('239.20.20.21', 10000)


def host_is_local(hostname, port):
    # https://gist.github.com/bennr01/7043a460155e8e763b3a9061c95faaa0
    """returns True if the hostname points to the localhost, otherwise False."""
    hostname = socket.getfqdn(hostname)
    if hostname in ('localhost', '0.0.0.0'):
        return True
    localhost = socket.gethostname()
    localaddrs = socket.getaddrinfo(localhost, port)
    targetaddrs = socket.getaddrinfo(hostname, port)
    for (family, socktype, proto, canonname, sockaddr) in localaddrs:
        for (rfamily, rsocktype, rproto, rcanonname, rsockaddr) in targetaddrs:
            if rsockaddr[0] == sockaddr[0]:
                return True
    return False


class EventBus:
    def __init__(self, name):
        if name is None:
            name = 'service'
        self._multicast_group = _g_multicast_group
        self._name = name
        self._quiet_count = 0
        self._mc_recv_sock = None
        self._mc_send_sock = None
        self._connect_recv_sock()
        self._connect_send_sock()
        loop = asyncio.get_event_loop()
        self._periodic_listen = Periodic(1, loop, self._listen_for_services)
        self._periodic_announce = Periodic(0.5, loop, self._announce_service)
        self._services = dict()
        self._state = dict()

    def _announce_service(self, n_periods):
        host, port = self._mc_send_sock.getsockname()
        announce = Announce()
        announce.host = socket.getfqdn(host)
        announce.port = port
        announce.service = self._name
        self._mc_send_sock.sendto(announce.SerializeToString(), self._multicast_group)

    def _listen_for_services(self, n_periods):
        if self._mc_recv_sock is None:
            self._quiet_count += 1
            if self._quiet_count == 3:
                self._connect_recv_sock()
                self._quiet_count = 0
        else:
            self._close_recv_sock()
            delete = []
            for key, service in self._services.items():
                if (time.time() - service.recv_stamp.ToSeconds()) > 10:
                    print('Dropping service: ', MessageToString(service, as_one_line=True))
                    delete.append(key)
                else:
                    print('Active service  : ', MessageToString(service, as_one_line=True))

            for key in delete:
                del self._services[key]

    def _close_recv_sock(self):
        asyncio.get_event_loop().remove_reader(self._mc_recv_sock.fileno())
        self._mc_recv_sock.close()
        self._mc_recv_sock = None

    def _connect_recv_sock(self):
        if self._mc_recv_sock is not None:
            asyncio.get_event_loop().remove_reader(self._mc_recv_sock.fileno())
            self._mc_recv_sock.close()
        self._mc_recv_sock = self._make_mc_recv_socket()
        asyncio.get_event_loop().add_reader(self._mc_recv_sock.fileno(), self._announce_recv)

    def _connect_send_sock(self):
        if self._mc_send_sock is not None:
            asyncio.get_event_loop().remove_reader(self._mc_send_sock.fileno())
            self._mc_send_sock.close()
        self._mc_send_sock = self._make_mc_send_socket()
        loop = asyncio.get_event_loop()
        loop.add_reader(self._mc_send_sock.fileno(), self._send_recv)

    def send(self, event: Event):
        self._state[event.name] = event
        buff = event.SerializeToString()
        for service in self._services.values():
            self._mc_send_sock.sendto(buff, (service.host, service.port))

    def _send_recv(self):
        data, server = self._mc_send_sock.recvfrom(1024)

        event = Event()
        event.ParseFromString(data)
        event.recv_stamp.GetCurrentTime()
        self._state[event.name] = event

        # print('received {!r} from {}'.format(
        # data, server))

    def _announce_recv(self):
        data, address = self._mc_recv_sock.recvfrom(1024)
        if address[1] == self._mc_send_sock.getsockname()[1] and host_is_local(address[0], address[1]):
            return

        announce = Announce()
        announce.ParseFromString(data)

        if address[1] != announce.port:
            logging.warning('announce port does not match sender... rejecting %s', MessageToString(announce, as_one_line=True))
        announce.host = address[0]
        announce.port = address[1]
        announce.recv_stamp.GetCurrentTime()
        self._services['%s:%d' % (announce.host, announce.port)] = announce

    def _make_mc_recv_socket(self):
        # Look up multicast group address in name server and find out IP version
        addrinfo = socket.getaddrinfo(self._multicast_group[0], None)[0]

        # Create the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind to the server address
        sock.bind(('', self._multicast_group[1]))

        group_bin = socket.inet_pton(addrinfo[0], addrinfo[4][0])

        # Join group
        if addrinfo[0] == socket.AF_INET:  # IPv4
            mreq = group_bin + struct.pack('=I', socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        else:
            mreq = group_bin + struct.pack('@I', 0)
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_JOIN_GROUP, mreq)
        return sock

    def _make_mc_send_socket(self):
        # Create the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Set the time-to-live for messages to 1 so they do not
        # go past the local network segment.
        ttl = struct.pack('b', 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        sock.bind(('', 0))
        return sock

    def get_last_event(self, name):
        return self._state.get(name, None)

    def log_state(self):
        logger.info('\n'.join([MessageToString(value, as_one_line=True) for value in self._state.values()]))


_g_event_bus = None


def get_event_bus(name=None):
    global _g_event_bus
    if _g_event_bus is None:
        _g_event_bus = EventBus(name)
    return _g_event_bus


def make_event(name: str, message, stamp: Timestamp = None) -> Event:
    event = Event()
    if stamp is None:
        event.stamp.GetCurrentTime()
    else:
        event.stamp.CopyFrom(stamp)
    event.name = name
    event.data.Pack(message)
    return event


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    event_bus = get_event_bus('ipc')
    _ = Periodic(1, event_loop, lambda n_periods: event_bus.log_state())

    event_loop.run_forever()


if __name__ == '__main__':
    main()
