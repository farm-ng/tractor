# socket_multicast_receiver.py
import socket
import struct
import sys
from farm_ng_proto.tractor.v1.io_pb2 import Event, Announce

from google.protobuf.text_format import MessageToString
from google.protobuf.timestamp_pb2 import Timestamp
import asyncio
import logging
from farm_ng.periodic import Periodic

import farm_ng_proto.tractor.v1.io_pb2
import farm_ng_proto.tractor.v1.motor_pb2
import farm_ng_proto.tractor.v1.tracking_camera_pb2
import farm_ng_proto.tractor.v1.steering_pb2

logger = logging.getLogger('ipc')
logger.setLevel(logging.INFO)


# https://en.wikipedia.org/wiki/Multicast_address
# adminstratively scoped: 239.0.0.0 to 239.255.255.255	
_g_multicast_group = ('239.20.20.20', 10000)


def host_is_local(hostname, port):
    # https://gist.github.com/bennr01/7043a460155e8e763b3a9061c95faaa0
    """returns True if the hostname points to the localhost, otherwise False."""
    hostname = socket.getfqdn(hostname)
    if hostname in ("localhost", "0.0.0.0"):
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
    def __init__(self):
        self._multicast_group = _g_multicast_group
        self._mc_recv_sock = self._make_mc_recv_socket()
        self._mc_send_sock = self._make_mc_send_socket()
        loop = asyncio.get_event_loop()
        loop.add_reader(self._mc_recv_sock.fileno(), self._bus_recv)
        loop.add_reader(self._mc_send_sock.fileno(), self._send_recv)
        self._state = dict()

    def send(self, event: Event):
        self._state[event.name] = event
        self._mc_send_sock.sendto(event.SerializeToString(), self._multicast_group)
        
    def _send_recv(self):
        data, server = self._mc_send_sock.recvfrom(1024)
        #print('received {!r} from {}'.format(
        #data, server))

    def _bus_recv(self):
        data, address = self._mc_recv_sock.recvfrom(1024)
        if address[1] == self._mc_send_sock.getsockname()[1] and host_is_local(address[0], address[1]):
            return
        
        #socket.getaddrinfo(*address)
        event = Event()
        event.ParseFromString(data)
        self._state[event.name] = event
        
        #print('my send addr {} received {} bytes from {} event {}'.format(
        #self._mc_send_sock.getsockname(),
        #len(data), address, MessageToString(event, as_one_line=True)))
        #self._mc_send_sock.sendto(b'ack %d'%len(data), address)


    def _make_mc_recv_socket(self):
        # Create the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Bind to the server address
        sock.bind(('',self._multicast_group[1]))

        # Tell the operating system to add the socket to
        # the multicast group on all interfaces.
        group = socket.inet_aton(self._multicast_group[0])
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            mreq)
        return sock


    def _send_announce(self):
        announce = Announce()
        self._send(announce)
        
        
    def _make_mc_send_socket(self):
        # Create the socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Set the time-to-live for messages to 1 so they do not
        # go past the local network segment.
        ttl = struct.pack('b', 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        return sock

    def get_last_event(self, name):
        return self._state.get(name, None)

    def log_state(self):
        logger.info('\n'.join([MessageToString(value, as_one_line=True) for value in self._state.values()]))

            
_g_event_bus = None
def get_event_bus():
    global _g_event_bus
    if _g_event_bus is None:
        _g_event_bus = EventBus()
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
    event_bus = get_event_bus()
    _ = Periodic(1, event_loop, lambda n_periods: event_bus.log_state())

    event_loop.run_forever()
    

if __name__ == "__main__":
    main()
    
