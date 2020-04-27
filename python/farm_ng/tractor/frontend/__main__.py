import argparse
import logging
import math
import os
import re
import time
import uuid
from typing import List
from typing import Set

import tornado.gen
import tornado.ioloop
import tornado.tcpclient
import tornado.web
import tornado.websocket

logger = logging.getLogger('fe')
logger.setLevel(logging.INFO)


class Application(tornado.web.Application):
    def __init__(self):
        handlers = [
            # GET request, called from root directory localhost:8080/
            (r'/', MainHandler),
            (r'/rtkroverstatussocket', RtkRoverSocketHandler),
            (r'/rtkroversolutionsocket', RtkRoverSolutionSocketHandler),
        ]

        settings = dict(
            cookie_secret='__TODO:_GENERATE_YOUR_OWN_RANDOM_VALUE_HERE__',
            template_path=os.path.join(os.path.dirname(__file__), 'templates'),
            static_path=os.path.join(os.path.dirname(__file__), 'static'),
            xsrf_cookies=True,
        )

        print(settings)
        print('template path:    ', settings['template_path'])
        print('static_path:    ', settings['static_path'])
        super().__init__(handlers, **settings)


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html', messages=RtkRoverSocketHandler.cache)


class RtkRoverSocketHandler(tornado.websocket.WebSocketHandler):
    waiters: Set[tornado.websocket.WebSocketHandler] = set()
    cache: List[str] = []
    cache_size = 200

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        RtkRoverSocketHandler.waiters.add(self)

    def on_close(self):
        RtkRoverSocketHandler.waiters.remove(self)

    @classmethod
    def update_cache(cls, status_msg):
        cls.cache.append(status_msg)
        if len(cls.cache) > cls.cache_size:
            cls.cache = cls.cache[-cls.cache_size:]

    @classmethod
    def send_updates(cls, status_msg):
        logger.debug('sending message to %d waiters', len(cls.waiters))
        for waiter in cls.waiters:
            try:
                waiter.write_message(status_msg)
            except RuntimeError as e:
                logger.error('Error sending message, %s', e, exc_info=True)

    @classmethod
    def new_message(cls, message_id, message):
        status_msg = {'id': message_id, 'body': message}
        status_msg['html'] = tornado.escape.to_basestring(
            '<div id="rtkrover_status"><pre>%s</pre></div>' % (message),
        )
        RtkRoverSocketHandler.update_cache(status_msg)
        RtkRoverSocketHandler.send_updates(status_msg)


class RtkRoverSolutionSocketHandler(tornado.websocket.WebSocketHandler):
    waiters: Set[tornado.websocket.WebSocketHandler] = set()

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        RtkRoverSolutionSocketHandler.waiters.add(self)

    def on_close(self):
        RtkRoverSolutionSocketHandler.waiters.remove(self)

    @classmethod
    def send_solution(cls, solution_msg):
        logger.debug('sending message to %d waiters', len(cls.waiters))
        for waiter in cls.waiters:
            try:
                waiter.write_message(solution_msg)
            except RuntimeError as e:
                logger.error('Error sending message, %s', e, exc_info=True)


def escape_ansi(line):
    ansi_escape = re.compile(r'(\x9B|\x1B\[)[0-?]*[ -\/]*[@-~]')
    return ansi_escape.sub('', line)


@tornado.gen.coroutine
def rtkrcv_telnet_loop(rtkrover_host):
    backoff = 0.25
    while True:
        try:
            yield tornado.gen.sleep(backoff)
            logger.info('connecting to rtkrover host: %s' % rtkrover_host)
            stream = yield tornado.tcpclient.TCPClient().connect(
                rtkrover_host, 2023, timeout=5,
            )
            logger.info('connected to rtkrover host: %s' % rtkrover_host)

            message = yield stream.read_until(b'password: ')
            message = message.strip(b'\xff\xfb\x03\xff\xfb\x01\r\n\x1b[1m')
            logger.info(message.decode('ascii'))
            yield stream.write(b'admin\r\n')
            message = yield stream.read_until(b'\r\n')
            logging.info(message.decode('ascii'))
            yield stream.write(b'status 1\r\n')
            backoff = 0.25  # reset backoff
            while True:
                message = yield stream.read_until(b'\x1b[2J')
                status_msg_ascii = escape_ansi(
                    message.rstrip(b'\x1b[2J').decode('ascii'),
                )
                RtkRoverSocketHandler.new_message(
                    str(uuid.uuid4()), status_msg_ascii,
                )
        except Exception as e:
            backoff = min(backoff*2, 10)
            exception_message = (
                '''Exception in rtkrover telnet comms
                {}\nretry in {:f} seconds {:f}'''.format(
                    e, backoff, time.time(),
                )
            )

            RtkRoverSocketHandler.new_message(
                str(uuid.uuid4()), exception_message,
            )
            logger.warning(exception_message)


@tornado.gen.coroutine
def rtkrcv_tcpcli_loop(rtkrover_host, rtkrover_tcpcli_port=9797):
    backoff = 0.25
    field_names = ['date']
    # % (e/n/u-baseline=WGS84,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=#
    # %  of satellites) UTC e-baseline(m) n-baseline(m) u-baseline(m)
    # %  Q ns sde(m) sdn(m) sdu(m) sden(m) sdnu(m) sdue(m) age(s)
    # %  ratio ve(m/s) vn(m/s) vu(m/s) sdve sdvn sdvu sdven sdvnu
    # %  sdvue
    status_map = {
        '1': 'fix', '2': 'float', '3': 'sbas',
        '4': 'dgps', '5': 'single', '6': 'ppp',
    }
    field_names = [
        'date', 'time_gps_utc', 'time_rcv_utc', 'e_baseline_m', 'n_baseline_m',
        'u_baseline_m', 'status', 'n_satelites', 'std_e_m', 'std_n_m',
        'std_u_m', 'sde_en_m', 'sde_nu_m', 'sde_ue_m', 'age', 'ar_ratio',
        'velocity_e_ms', 'velocity_n_ms', 'velocity_u_ms', 'std_ve',
        'std_vn', 'std_vu', 'std_ven', 'std_vnu', 'std_vue',
    ]
    while True:
        try:
            yield tornado.gen.sleep(backoff)
            logger.info(
                'connecting to rtkrover tcpcli out: %s:%s',
                rtkrover_host, rtkrover_tcpcli_port,
            )
            stream = yield tornado.tcpclient.TCPClient().connect(
                rtkrover_host, rtkrover_tcpcli_port, timeout=5,
            )
            logger.info(
                'connected to rtkrover tcpcli out: %s:%s',
                rtkrover_host, rtkrover_tcpcli_port,
            )
            backoff = 0.25  # reset backoff
            while True:
                message = yield stream.read_until(b'\n')

                # HACKY, but adding a field for receive time to get a
                # sense of latency between GPS position and when we
                # got this message.  the clock on the tractor is
                # syncronized by NTP currently, so that has some error
                # TODO(ethanrublee) figure out how to use the GPS in
                # ntp.
                time_seconds = time.time()
                ts = time.strftime(
                    '%H:%M:%S', time.gmtime(
                        time_seconds,
                    ),
                )+(
                    '%0.3f' % (
                        time_seconds -
                        math.floor(time_seconds)
                    )
                ).lstrip('0')
                message = message.rstrip(b'\n').decode('ascii')
                fields = re.split(r'\s+', message)
                # expecting fields to be the correct number,
                # TODO(ethanrublee) better input checking...
                assert len(fields) == len(field_names) - \
                    1, '%d != %d' % (len(fields), len(field_names)-1)
                # inserting it into the position after time_gps_utc,
                # with the field name time_rcv_utc
                fields.insert(2, ts)
                assert len(fields) == len(field_names), '%d != %d' % (
                    len(fields), len(field_names),
                )

                # create a dict of field names to field values
                gps_state = dict(zip(field_names, fields))
                # replace the status integer with the string name
                gps_state['status'] = status_map[gps_state['status']]

                RtkRoverSolutionSocketHandler.send_solution(gps_state)
                logger.debug('message %s', gps_state)
        except Exception as e:
            backoff = min(backoff*2, 10)
            exception_message = (
                '''Exception in rtkrover tcpcli comms
                {}\nretry in {:f} seconds {:f}'''.format(
                    e, backoff, time.time(),
                )
            )
            logger.warning(exception_message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rtkrover-host', default='localhost')
    args = parser.parse_args()
    app = Application()
    app.listen(8080)
    loop = tornado.ioloop.IOLoop.current()
    loop.spawn_callback(rtkrcv_telnet_loop, args.rtkrover_host)
    loop.spawn_callback(rtkrcv_tcpcli_loop, args.rtkrover_host)
    loop.start()


if __name__ == '__main__':
    main()
