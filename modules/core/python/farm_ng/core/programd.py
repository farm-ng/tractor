import asyncio
import logging
import os

import google.protobuf.json_format as json_format

from farm_ng.core.ipc import EventBus
from farm_ng.core.ipc import EventBusQueue
from farm_ng.core.ipc import get_event_bus
from farm_ng.core.ipc import get_message
from farm_ng.core.ipc import make_event
from farm_ng.core.programd_pb2 import ProgramdConfig
from farm_ng.core.programd_pb2 import ProgramSupervisorStatus
from farm_ng.core.programd_pb2 import StartProgramRequest
from farm_ng.core.programd_pb2 import StopProgramRequest

logger = logging.getLogger('programd')
logger.setLevel(logging.INFO)

farm_ng_root = os.environ['FARM_NG_ROOT']


class ProgramSupervisor:
    def __init__(self, event_bus: EventBus):
        self._config = ProgramdConfig()

        # For now, load the core program config we check into the source tree.
        # In the future, could replace or augment, with program config from the blobstore.
        with open(os.path.join(farm_ng_root, 'modules/core/config/programd/programs.json')) as f:
            json_format.Parse(f.read(), self._config)

        self._event_bus = event_bus
        self._event_bus.add_subscriptions(['programd/request'])
        self.status = ProgramSupervisorStatus(
            stopped=ProgramSupervisorStatus.ProgramStopped(),
            library=self._config.programs,
        )
        self.shutdown = False
        self.child_process = None

    async def run(self):
        await asyncio.gather(self.send_status(), self.handle_start(), self.handle_stop())

    async def send_status(self):
        while not self.shutdown:
            event = make_event('programd/status', self.status)
            self._event_bus.send(event)
            await asyncio.sleep(1)

    async def handle_stop(self):
        with EventBusQueue(self._event_bus) as event_queue:
            while not self.shutdown:
                stop_request: StopProgramRequest = await get_message(
                    event_queue,
                    'programd/request',
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
                start_request: StartProgramRequest = await get_message(event_queue, 'programd/request', StartProgramRequest)
                if self.status.WhichOneof('status') != 'stopped':
                    logger.info(f"StartProgramRequest received while program status was {self.status.WhichOneof('status')}")
                    continue
                program_info = next((p for p in self._config.programs if p.id == start_request.id), None)
                if not program_info:
                    logger.info(f'StartProgramRequest received for program {start_request.id} which does not exist.')
                    continue
                self.status.running.program.id = start_request.id
                asyncio.get_event_loop().create_task(self.launch_child_process(program_info))

    async def launch_child_process(self, program_info):
        launch_path = os.path.join(
            os.environ.get(program_info.launch_path.root_env_var, ''),
            program_info.launch_path.path,
        )
        logger.info('Launching ', launch_path, program_info.launch_args)
        self.child_process = await asyncio.create_subprocess_exec(launch_path, *program_info.launch_args)
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
    event_bus = get_event_bus('programd')
    supervisor = ProgramSupervisor(event_bus)
    event_bus.event_loop().run_until_complete(supervisor.run())
