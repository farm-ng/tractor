import argparse
import asyncio
import logging
import sys

from farm_ng.ipc import AnnounceQueue
from farm_ng.ipc import EventBus
from farm_ng.ipc import EventBusQueue
from farm_ng.ipc import get_message
from farm_ng.ipc import make_event
from farm_ng_proto.tractor.v1.calibrator_pb2 import CalibratorCommand
from farm_ng_proto.tractor.v1.calibrator_pb2 import CalibratorStatus
from farm_ng_proto.tractor.v1.io_pb2 import LoggingCommand
from farm_ng_proto.tractor.v1.io_pb2 import LoggingStatus
from farm_ng_proto.tractor.v1.tracking_camera_pb2 import TrackingCameraCommand
from google.protobuf.text_format import MessageToString

logging.basicConfig(stream=sys.stdout, level=logging.INFO)

parser = argparse.ArgumentParser()
parser.add_argument('name')
parser.add_argument('--tag_ids', nargs='+', default=[221, 226, 225, 218, 222])
parser.add_argument('--num_frames', type=int, default=2)
args = parser.parse_args()

event_bus = EventBus('logging_client')


async def wait_for_services(event_bus, desired_services: set):
    logging.info('Waiting for services: %s', desired_services)
    services = set()
    with AnnounceQueue(event_bus) as announce_queue:

        while True:
            announce = await announce_queue.get()
            services.add(announce.service)
            if services.issuperset(desired_services):
                break
    logging.info('Have services: %s', services)


async def wait_for_predicate(event_queue, name, message_type, predicate):
    while True:
        message = await get_message(event_queue, name, message_type)
        logging.info('message: %s', MessageToString(message, as_one_line=True))
        if predicate(message):
            return message


async def start_recording(event_queue):
    await wait_for_services(event_bus, ['ipc-logger', 'tracking-camera', 'calibrator'])
    event_bus.send(
        make_event(
            'logger/command',
            LoggingCommand(
                record_start=dict(
                    archive_path=args.name,
                    mode=LoggingCommand.RecordStart.MODE_ALL_MESSAGES,
                ),
            ),
        ),
    )

    await wait_for_predicate(
        event_queue, 'logger/status', LoggingStatus,
        lambda status: status.HasField('recording'),
    )

    event_bus.send(
        make_event(
            'tracking_camera/command',
            TrackingCameraCommand(record_start=dict(mode=TrackingCameraCommand.RecordStart.MODE_APRILTAG_STABLE)),
        ),
    )


async def stop_recording(event_queue):
    event_bus.send(
        make_event(
            'tracking_camera/command',
            TrackingCameraCommand(record_stop={}),
        ),
    )

    event_bus.send(
        make_event(
            'logger/command',
            LoggingCommand(record_stop={}),
        ),
    )

    await wait_for_predicate(
        event_queue, 'logger/status', LoggingStatus,
        lambda status: status.HasField('stopped'),
    )


async def calibrate(event_queue):

    event_bus.send(
        make_event(
            'calibrator/command',
            CalibratorCommand(apriltag_rig_start=dict(tag_ids=args.tag_ids)),
        ),
    )

    await wait_for_predicate(
        event_queue, 'calibrator/status', CalibratorStatus,
        lambda status: (
            status.HasField('apriltag_rig') and
            status.apriltag_rig.num_frames >= args.num_frames
        ),
    )

    event_bus.send(
        make_event(
            'calibrator/command',
            CalibratorCommand(solve={}),
        ),
    )

    return await wait_for_predicate(
        event_queue, 'calibrator/status', CalibratorStatus,
        lambda status: (
            status.HasField('apriltag_rig') and
            status.apriltag_rig.HasField('rig_model_resource') and
            status.apriltag_rig.finished
        ),
    )


with EventBusQueue(event_bus) as event_queue:
    try:
        event_loop = asyncio.get_event_loop()
        event_loop.run_until_complete(start_recording(event_queue))
        event_loop.run_until_complete(calibrate(event_queue))
    finally:
        event_loop.run_until_complete(stop_recording(event_queue))
