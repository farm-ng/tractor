# socket_multicast_receiver.py
import asyncio
import logging
import socket
import struct
import sys
import time

import farm_ng.proto_utils  # noqa: F401
from farm_ng.periodic import Periodic
from farm_ng_proto.tractor.v1.io_pb2 import Announce
from farm_ng_proto.tractor.v1.io_pb2 import Event
from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp

# loads all the protos for pretty print of any

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


class EventBusQueue:
    def __init__(self, event_bus):
        self._event_bus = event_bus

    def __enter__(self):
        logger.info('adding event queue')
        self._queue = self._event_bus._queue()
        return self._queue

    def __exit__(self, type, vlaue, traceback):
        logger.info('removing event queue')
        self._event_bus._remove_queue(self._queue)


class EventBus:
    def __init__(self, name, recv_raw=False):
        if name is None:
            name = 'python-ipc'
        # forward raw packets, don't track state
        self._recv_raw = recv_raw
        self._multicast_group = _g_multicast_group
        self._name = name
        self._quiet_count = 0
        self._mc_recv_sock = None
        self._mc_send_sock = None
        self._connect_recv_sock()
        self._connect_send_sock()
        loop = asyncio.get_event_loop()
        self._periodic_listen = Periodic(2, loop, self._listen_for_services)
        self._periodic_announce = Periodic(1, loop, self._announce_service)
        self._services = dict()
        self._state = dict()
        self._subscribers = set()

    def _announce_service(self, n_periods):
        host, port = self._mc_send_sock.getsockname()
        announce = Announce()
        announce.stamp.GetCurrentTime()
        # For now, only announce our local address
        # announce.host = socket.getfqdn(host) # this may take a while
        announce.host = '127.0.0.1'
        announce.port = port
        announce.service = self._name
        msg = announce.SerializeToString()
        self._mc_send_sock.sendto(msg, self._multicast_group)

    def _queue(self):
        queue = asyncio.Queue()
        self._subscribers.add(queue)
        return queue

    def _remove_queue(self, queue):
        self._subscribers.remove(queue)

    def _listen_for_services(self, n_periods):
        if self._mc_recv_sock is None:
            self._quiet_count += 1
            if self._quiet_count == 3:
                self._connect_recv_sock()
                logger.info('Listening for services.')
                self._quiet_count = 0
        else:
            self._close_recv_sock()
            logger.info('Resting for services.')
            delete = []
            for key, service in self._services.items():
                if (time.time() - service.recv_stamp.ToSeconds()) > 10:
                    logger.info('Dropping service: %s', MessageToString(service, as_one_line=True))
                    delete.append(key)
                else:
                    logger.info('Active service  : %s', MessageToString(service, as_one_line=True))

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
        if self._recv_raw:
            for q in self._subscribers:
                q.put_nowait(data)
            return

        event = Event()
        event.ParseFromString(data)
        event.recv_stamp.GetCurrentTime()

        self._state[event.name] = event

        for q in self._subscribers:
            q.put_nowait(event)

    def _announce_recv(self):
        data, address = self._mc_recv_sock.recvfrom(1024)

        # Ignore self-announcements
        if address[1] == self._mc_send_sock.getsockname()[1]:
            return

        # Ignore non-local announcements
        if not host_is_local(address[0], address[1]):
            logger.warning('ignoring non-local announcement: %s:%s', address[0], address[1])
            return

        announce = Announce()
        announce.ParseFromString(data)

        # Ignore faulty announcements
        if announce.host != '127.0.0.1' or announce.port != address[1]:
            logger.warning('announcement does not match sender... rejecting %s', MessageToString(announce, as_one_line=True))
            return

        # Store the announcement
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

        # Set the time-to-live for messages to 0 so they do not
        # leave localhost.
        ttl = struct.pack('b', 0)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
        sock.bind(('', 0))
        return sock

    def get_last_event(self, name):
        assert not self._recv_raw, 'EventBus initialized with recv_raw, no state.'
        return self._state.get(name, None)

    def log_state(self):
        assert not self._recv_raw, 'EventBus initialized with recv_raw, no state.'
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
    event_bus = get_event_bus('python-ipc')
    _ = Periodic(1, event_loop, lambda n_periods: event_bus.log_state())

    event_loop.run_forever()


if __name__ == '__main__':
    main()
