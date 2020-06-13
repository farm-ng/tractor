# TODO: Make pylint handle protobuf generated code properly, possibly with pylint-protobuf
# pylint: disable=no-member
import asyncio
import glob
import logging
import time

import proio
import tornado.ioloop
import tornado.platform.asyncio
import tornado.tcpclient
import tornado.web
import tornado.websocket
from farm_ng_proto.tractor.v1 import motor_pb2
from farm_ng_proto.tractor.v1 import playback_service_pb2
from farm_ng_proto.tractor.v1 import rtk_pb2
from gensrv.farm_ng_proto.tractor.v1.playback_service_twirp_srv import PlaybackServiceImpl
from gensrv.farm_ng_proto.tractor.v1.playback_service_twirp_srv import PlaybackServiceRequestHandler
from google.protobuf.timestamp_pb2 import Timestamp

logger = logging.getLogger('playback')
logger.setLevel(logging.INFO)


def get_stamp_from_event(event: proio.Event) -> Timestamp:
    part = event.tagged_entries('stamp')
    assert len(part) == 1
    stamp = event.get_entry(part[0])
    return stamp


class PlaybackTractor:
    def __init__(self):
        self._left_motor_state = motor_pb2.MotorControllerState()
        self._right_motor_state = motor_pb2.MotorControllerState()
        self._rtk_solution = rtk_pb2.RtkSolution()

    def right_motor_state(self, tag: str, stamp: Timestamp, x: motor_pb2.MotorControllerState):
        print('right motor', stamp.ToNanoseconds()*1e-9)
        self._right_motor_state.MergeFrom(x)

    def left_motor_state(self, tag: str, stamp: Timestamp, x: motor_pb2.MotorControllerState):
        print('left motor', stamp.ToNanoseconds()*1e-9)
        self._left_motor_state.MergeFrom(x)

    def rtk_solution(self, tag: str, stamp: Timestamp, x: rtk_pb2.RtkSolution):
        print('rtk_sol', stamp.ToNanoseconds()*1e-9)
        self._rtk_solution.CopyFrom(x)

    def _default_handler(self, tag: str, stamp: Timestamp, x):
        print('default handler tag({}) stamp({:0.6f}): {}\n'.format(tag, stamp.ToNanoseconds()*1e-9, type(x)))

    def __call__(self, event: proio.Event):
        '''
        Called on each event.
        '''
        stamp = get_stamp_from_event(event)
        for tag in event.tags():
            handler = getattr(self, tag.replace('/', '_'), self._default_handler)
            part = event.tagged_entries(tag)
            if tag == 'stamp':
                continue
            for entry in part:
                x = event.get_entry(entry)
                handler(tag, stamp, x)


class PlaybackService(PlaybackServiceImpl):
    def __init__(self):
        self.log_path = None
        self.reader = None
        self.play_task = None
        self.playback_hook = PlaybackTractor()

    async def playback_loop(self, play_request: playback_service_pb2.PlayRequest):
        assert self.reader is not None
        print(play_request)
        self.reader.seek_to_start()
        self.reader.skip(play_request.start_event)
        count = 0
        t0_event = next(self.reader)
        t0: Timestamp = get_stamp_from_event(t0_event)
        t_estart = t0
        t_pstart = time.time()

        realtime = play_request.playback_rate == playback_service_pb2.PlayRequest.RATE_REALTIME
        while True:
            try:
                self.playback_hook(t0_event)
            except Exception as e:
                print(e)

            count += 1
            if count >= play_request.n_events and play_request.n_events > 0:
                break

            try:
                t1_event = next(self.reader)
            except StopIteration:
                break

            t1: Timestamp = get_stamp_from_event(t1_event)
            sleep_time: float = 0.0  # asap sleep time
            if realtime:
                event_time = t_pstart + (t1.ToNanoseconds() - t_estart.ToNanoseconds())*1.e-9
                sleep_time = event_time - time.time()
            t0_event = t1_event
            t0 = t1
            await asyncio.sleep(sleep_time)

        t_eend = t0
        t_pend = time.time()
        print('event duration %f' % ((t_eend.ToNanoseconds() - t_estart.ToNanoseconds())*1e-9))
        print('play duration %f' % (t_pend - t_pstart))

        if play_request.loop:
            print('loop')
            self.play_task = asyncio.get_event_loop().create_task(self.playback_loop(play_request))

    def ListLogs(self, list_logs_request: playback_service_pb2.ListLogsRequest) -> playback_service_pb2.ListLogsResponse:
        print('ListLogs', list_logs_request)
        response = playback_service_pb2.ListLogsResponse()
        paths = []
        for path in glob.glob('/tmp/farm-ng-logs/*.proio'):
            print(path)
            paths.append(path)
        response.log_paths[:] = paths[:]
        return response

    def OpenLog(self, open_log_request: playback_service_pb2.OpenLogRequest) -> playback_service_pb2.OpenLogResponse:
        reader = proio.Reader(open_log_request.log_path)
        n_events = 0

        while True:
            skipped = reader.skip(1000)
            n_events += skipped
            if skipped != 1000:
                break

        if n_events > 0:
            reader.seek_to_start()
            first_event: proio.Event = next(reader)

        last_event: proio.Event = None
        if n_events > 1:
            reader.seek_to_start()
            reader.skip(n_events-1)
            last_event = next(reader)
        else:
            last_event = first_event
        reader.seek_to_start()
        if self.reader is not None:
            self.reader.close()

        self.reader = reader
        self.log_path = open_log_request.log_path
        self.open_log_response = playback_service_pb2.OpenLogResponse(
            n_events=n_events,
            stamp_begin=get_stamp_from_event(first_event),
            stamp_end=get_stamp_from_event(last_event),
        )
        return self.open_log_response

    def Play(self, play_request: playback_service_pb2.PlayRequest) -> playback_service_pb2.PlayResponse:
        if self.play_task is not None:
            self.play_task.cancel()
        self.play_task = asyncio.get_event_loop().create_task(self.playback_loop(play_request))
        return playback_service_pb2.PlayResponse()

    def Stop(self, stop_request: playback_service_pb2.StopRequest) -> playback_service_pb2.StopResponse:
        if self.play_task is not None:
            self.play_task.cancel()
        return playback_service_pb2.StopResponse()


class Application(tornado.web.Application):
    def __init__(self):
        handlers = []

        # Add Twirp services
        for service, handler in [(PlaybackService, PlaybackServiceRequestHandler)]:
            handlers.append((rf'{handler.prefix}/.*/?', handler, {'service': service()}))

        settings = dict(
            debug=True,
        )
        super().__init__(handlers, **settings)


def exception_handler(x):
    print(x)


def main():
    ioloop = asyncio.get_event_loop()
    ioloop.set_debug(True)
    tornado.platform.asyncio.AsyncIOMainLoop().install()

    app = Application()
    app.listen(9011)
    ioloop.run_forever()


if __name__ == '__main__':
    main()
