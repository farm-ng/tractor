import asyncio
import logging

import farm_ng.proto_utils  # noqa: F401 # loads all the protos for pretty print of any
import websockets
from farm_ng_proto.tractor.v1.io_pb2 import Event
from google.protobuf.text_format import MessageToString


logging.basicConfig()


async def ipc_recv():
    uri = 'ws://localhost:8989'
    async with websockets.connect(uri) as websocket:
        while True:
            msg = await websocket.recv()
            event = Event()
            event.ParseFromString(msg)
            print(MessageToString(event, as_one_line=True))

asyncio.get_event_loop().run_until_complete(ipc_recv())
