import asyncio
import json
import logging
import sys

import websockets
from farm_ng.ipc import EventBus
from farm_ng.ipc import EventBusQueue

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

logger = logging.getLogger('ipc_websocket_bridge')
logger.setLevel(logging.INFO)


event_bus = EventBus('ipc_websocket', recv_raw=True)


async def ws_event_bus(websocket, path):
    with EventBusQueue(event_bus) as event_queue:
        while True:
            msg = await event_queue.get()
            await websocket.send(msg)


start_server = websockets.serve(ws_event_bus, 'localhost', 8989)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
