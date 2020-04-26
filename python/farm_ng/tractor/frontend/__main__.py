import argparse
import logging
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
    waiters: Set[tornado.websockethandler.Websockethandler] = set()
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rtkrover-host', default='localhost')
    args = parser.parse_args()
    app = Application()
    app.listen(8080)
    loop = tornado.ioloop.IOLoop.current()
    loop.spawn_callback(rtkrcv_telnet_loop, args.rtkrover_host)
    loop.start()


if __name__ == '__main__':
    main()
