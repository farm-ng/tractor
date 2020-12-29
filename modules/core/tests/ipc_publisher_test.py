import asyncio
import collections
import os
import struct
import unittest

from google.protobuf.timestamp_pb2 import Timestamp

from farm_ng.core.blobstore import Blobstore
from farm_ng.core.io_pb2 import Event
from farm_ng.core.io_pb2 import LoggingCommand
from farm_ng.core.ipc import get_event_bus
from farm_ng.core.ipc import make_event
from farm_ng.core.resource_pb2 import BUCKET_LOGS

event_bus = get_event_bus('ipc-publisher')
event_bus.add_subscriptions(['.*'])

N_MESSAGES = 5
MESSAGE_NAME_PREFIX = 'ipc-publisher/status'
ARCHIVE_NAME = 'default'
BLOBSTORE = Blobstore()


def start_logging():
    event_bus.send(
        make_event(
            'logger/command', LoggingCommand(
                record_start=LoggingCommand.RecordStart(
                    archive_path=os.path.join(BLOBSTORE.bucket_relative_path(BUCKET_LOGS), ARCHIVE_NAME),
                ),
            ),
        ),
    )


async def run():
    # TODO(isherman): WaitForServices instead
    await asyncio.sleep(3)

    start_logging()

    for i in range(N_MESSAGES):
        event_bus.send(make_event(f'{MESSAGE_NAME_PREFIX}/{i}', Timestamp()))
        await asyncio.sleep(1)


def event_counts_by_name(log_path):
    counter = collections.Counter()
    with open(log_path, 'rb') as f:
        while True:
            header = f.read(2)
            if header == b'':
                break
            size, = struct.unpack('<H', header)
            message = f.read(size)
            event = Event()
            event.ParseFromString(message)
            counter[event.name] += 1
    return counter


class Test(unittest.TestCase):
    def test(self):
        logs_path = os.path.join(BLOBSTORE.root, BLOBSTORE.bucket_relative_path(BUCKET_LOGS), ARCHIVE_NAME)
        self.assertTrue(os.path.isdir(logs_path), logs_path)
        logs = os.listdir(logs_path)
        self.assertEqual(len(logs), 1)
        counter = event_counts_by_name(os.path.join(logs_path, logs[0]))

        for i in range(N_MESSAGES):
            self.assertEqual(counter[f'{MESSAGE_NAME_PREFIX}/{i}'], 1)


def main():
    event_bus.event_loop().run_until_complete(run())
    unittest.main()


if __name__ == '__main__':
    main()
