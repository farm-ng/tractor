import atexit
import os
import time
from typing import Dict

import proio
from google.protobuf.timestamp_pb2 import Timestamp


class ProioLogger:
    _writer = None

    def __init__(self, log_dir='/tmp/farm-ng-%s'% time.strftime('%Y-%m-%d-%H-%M-%S')):
        self.log_dir = log_dir
        os.makedirs(self.log_dir)
        self._log_file = os.path.join(log_dir, 'log.proio')
        self._writer = None

    def _close(self):
        if self._writer is not None:
            self._writer.close()

    def __del__(self):
        self._close()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self._close()

    def make_event(self, entries: Dict, stamp: Timestamp = None) -> proio.Event:
        if stamp is None:
            stamp = Timestamp()
            stamp.GetCurrentTime()
        event = proio.Event()
        event.add_entry('stamp', stamp)
        if entries is not None:
            for tag, value in entries.items():
                event.add_entry(tag, value)
        return event

    def writer(self):
        if self._writer is None:
            self._writer = proio.Writer(filename=self._log_file)
        return self._writer


_proi_logger = None


def get_proio_logger():
    global _proi_logger
    if _proi_logger is None:
        _proi_logger = ProioLogger()
    return _proi_logger


@atexit.register
def _close_proio_logger():
    if _proi_logger is not None:
        _proi_logger._close()
        print('closing plog')


def smoke1():
    from farm_ng_proto.tractor.v1.geometry_pb2 import Vec2  # for testing
    import numpy as np

    with ProioLogger() as plog:
        plog.writer().push_metadata('run', b'smoke1')
        print(plog._log_file)

        v1 = Vec2()

        v2 = Vec2()
        for alpha in np.linspace(0, 2*np.pi, 10):
            time.sleep(0.1)
            v1.x = np.sin(alpha)
            v1.y = np.cos(alpha)
            v2.x = np.sin(2*alpha)
            v2.y = np.cos(2*alpha)
            event = plog.make_event({'v1': v1, 'v2': v2})
            plog.writer().push(event)

    with proio.Reader(plog._log_file) as reader:
        for evnum, event in enumerate(reader):
            v1_ids = event.tagged_entries('v1')
            assert len(v1_ids) == 1
            v1 = event.get_entry(v1_ids[0])
            print(f'v1 {v1.x:f},{v1.y:f}')

            v2_ids = event.tagged_entries('v2')
            assert len(v2_ids) == 1
            v2 = event.get_entry(v2_ids[0])

            print(f'v2 {v2.x:f},{v2.y:f}')


if __name__ == '__main__':
    smoke1()
