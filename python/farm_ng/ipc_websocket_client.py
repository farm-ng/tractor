import asyncio
import json
import logging

import farm_ng_proto.tractor.v1.io_pb2
import farm_ng_proto.tractor.v1.motor_pb2
import farm_ng_proto.tractor.v1.steering_pb2
import farm_ng_proto.tractor.v1.tracking_camera_pb2
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
