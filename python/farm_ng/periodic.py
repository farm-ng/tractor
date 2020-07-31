import logging

import linuxfd

logger = logging.getLogger('periodic')
logger.setLevel(logging.INFO)


class Periodic:
    def __init__(self, period_seconds, event_loop, callback, name=''):
        # rtc=False means a monotonic clock for realtime loop as it won't
        # be adjusted by the system admin
        self.name = name
        self.timer = linuxfd.timerfd(rtc=False, nonBlocking=True)
        # here we start a timer that will fire in one second, and then
        # each command period after that
        self.period_seconds = period_seconds
        self.callback = callback
        self.event_loop = event_loop
        self.timer.settime(value=1.0, interval=self.period_seconds)
        self.event_loop.add_reader(self.timer, self._read_timer)

    def _read_timer(self):
        n_periods = self.timer.read()
        if n_periods > 1:
            logger.warning('%s period: n_periods %d skipped!', self.name, n_periods - 1)
        self.callback(n_periods)
