import asyncio
import logging
import os
from collections import namedtuple

from farm_ng.ipc import EventBus
from farm_ng.ipc import EventBusQueue
from farm_ng.ipc import get_event_bus
from farm_ng.ipc import get_message
from farm_ng.ipc import make_event
from farm_ng_proto.tractor.v1.program_supervisor_pb2 import Program
from farm_ng_proto.tractor.v1.program_supervisor_pb2 import ProgramSupervisorStatus
from farm_ng_proto.tractor.v1.program_supervisor_pb2 import StartProgramRequest
from farm_ng_proto.tractor.v1.program_supervisor_pb2 import StopProgramRequest

logger = logging.getLogger('program_supervisor')
logger.setLevel(logging.INFO)

farm_ng_root = os.environ['FARM_NG_ROOT']

ProgramInfo = namedtuple('ProgramInfo', 'path args name description')

library = {
    'calibrate_apriltag_rig_playback': ProgramInfo(
        path=f'{farm_ng_root}/build/cpp/farm_ng/log_playback',
        args=['-send', '-log', f'{farm_ng_root}/../tractor-data/cal01/events-02498-00000.log'],
        name='Apriltag Rig Calibration Playback',
        description='Log playback',
    ),
    'capture_calibration_dataset': ProgramInfo(
        path=f'{farm_ng_root}/build/cpp/farm_ng/capture_calibration_dataset',
        args=['-interactive'],
        name='Capture Calibration Dataset',
        description='Capture apriltag detections, for use in other calibration programs',
    ),
    'calibrate_apriltag_rig': ProgramInfo(
        path=f'{farm_ng_root}/build/cpp/farm_ng/calibrate_apriltag_rig',
        args=['-interactive'],
        name='Apriltag Rig Calibration',
        description='Solves an apriltag rig from data collected with capture_calibration_dataset',
    ),
    'calibrate_base_to_camera': ProgramInfo(
        path=f'{farm_ng_root}/build/cpp/farm_ng/calibrate_base_to_camera',
        args=['-interactive'],
        name='Base-to-Camera Calibration',
        description=(
            'Solves a base_pose_camera and other base calibration parameters from '
            'an apriltag rig and data collected with capture_calibration_dataset'
        ),
    ),
    'capture_video_dataset': ProgramInfo(
        path=f'{farm_ng_root}/build/cpp/farm_ng/capture_video_dataset',
        args=['-interactive'],
        name='Capture Video Dataset',
        description='Capture video segments, for use in other programs',
    ),
    'calibrate_multi_view_apriltag_rig': ProgramInfo(
        path=f'{farm_ng_root}/build/cpp/farm_ng/calibrate_multi_view_apriltag_rig',
        args=['-interactive'],
        name='Multi View Apriltag Rig Calibration',
        description='Solves a multiview apriltag rig from data collected with capture_calibration_dataset',
    ),
    'sleep-5': ProgramInfo(path='sleep', args=['5'], name='Sleep 5', description='Take a nap'),
}
libraryPb = [Program(id=_id, name=p.name, description=p.description) for _id, p in library.items()]


class ProgramSupervisor:
    def __init__(self, event_bus: EventBus):
        self._event_bus = event_bus
        self._event_bus.add_subscriptions(['program_supervisor/request'])
        self.status = ProgramSupervisorStatus(stopped=ProgramSupervisorStatus.ProgramStopped(), library=libraryPb)
        self.shutdown = False
        self.child_process = None

    async def run(self):
        await asyncio.gather(self.send_status(), self.handle_start(), self.handle_stop())

    async def send_status(self):
        while not self.shutdown:
            event = make_event('program_supervisor/status', self.status)
            self._event_bus.send(event)
            await asyncio.sleep(1)

    async def handle_stop(self):
        with EventBusQueue(self._event_bus) as event_queue:
            while not self.shutdown:
                stop_request: StopProgramRequest = await get_message(
                    event_queue,
                    'program_supervisor/request',
                    StopProgramRequest,
                )
                if self.status.WhichOneof('status') != 'running':
                    logger.info(f"StopProgramRequest received while program status was {self.status.WhichOneof('status')}")
                    continue

                if stop_request.id != self.status.running.program.id:
                    logger.info(f'StopProgramRequest received for program {stop_request.id} while program {self.status.running.program.id} was running')
                    continue

                self.child_process.terminate()

    async def handle_start(self):
        with EventBusQueue(self._event_bus) as event_queue:
            while not self.shutdown:
                start_request: StartProgramRequest = await get_message(event_queue, 'program_supervisor/request', StartProgramRequest)
                if self.status.WhichOneof('status') != 'stopped':
                    logger.info(f"StartProgramRequest received while program status was {self.status.WhichOneof('status')}")
                    continue
                program_info = library.get(start_request.id)
                if not program_info:
                    logger.info(f'StartProgramRequest received for program {start_request.id} which does not exist.')
                    continue
                self.status.running.program.id = start_request.id
                asyncio.get_event_loop().create_task(self.launch_child_process(program_info))

    async def launch_child_process(self, program_info):
        logger.info('Launching ', program_info.path, program_info.args)
        self.child_process = await asyncio.create_subprocess_exec(program_info.path, *program_info.args)
        self.status.running.program.pid = self.child_process.pid
        self.status.running.program.stamp_start.GetCurrentTime()
        await self.monitor_child_process()

    async def monitor_child_process(self):
        await self.child_process.wait()
        self.status.stopped.last_program.CopyFrom(self.status.running.program)
        self.status.stopped.last_program.stamp_end.GetCurrentTime()
        self.status.stopped.last_program.exit_code = self.child_process.returncode

        self.child_process = None


if __name__ == '__main__':
    event_bus = get_event_bus('program_supervisor')
    supervisor = ProgramSupervisor(event_bus)
    event_bus.event_loop().run_until_complete(supervisor.run())
