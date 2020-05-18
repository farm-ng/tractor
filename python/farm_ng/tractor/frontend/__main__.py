import argparse
import asyncio
import logging
import os
import uuid
from typing import List
from typing import Set

import tornado.gen
import tornado.ioloop
import tornado.tcpclient
import tornado.web
import tornado.websocket
from farm_ng.rtkcli import RtkClient

logger = logging.getLogger('fe')
logger.setLevel(logging.INFO)


class Application(tornado.web.Application):
    def __init__(self):
        third_party_path = os.path.join(
            os.environ['FARM_NG_ROOT'],
            'jsm/third_party',
        )

        handlers = [
            # GET request, called from root directory localhost:8080/
            (r'/', MainHandler),
            (
                r'/third_party/(.*)', tornado.web.StaticFileHandler,
                dict(path=third_party_path),
            ),
            (r'/mapper', MapperHandler),
            (r'/rtkroverstatussocket', RtkRoverSocketHandler),
            (r'/rtkroversolutionsocket', RtkRoverSolutionSocketHandler),
        ]

        settings = dict(
            cookie_secret='__TODO:_GENERATE_YOUR_OWN_RANDOM_VALUE_HERE__',
            template_path=os.path.join(os.path.dirname(__file__), 'templates'),
            static_path=os.path.join(os.path.dirname(__file__), 'static'),
            xsrf_cookies=True,
            debug=True,
        )

        print(settings)
        print('template path:    ', settings['template_path'])
        print('static_path:    ', settings['static_path'])
        super().__init__(handlers, **settings)


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html', messages=RtkRoverSocketHandler.cache)


class MapperHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('mapper.html')


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
    def new_message(cls, message):
        message_id = str(uuid.uuid4())
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


def main():
    tornado.platform.asyncio.AsyncIOMainLoop().install()
    loop = asyncio.get_event_loop()
    parser = argparse.ArgumentParser()
    parser.add_argument('--rtkrover-host', default='localhost')
    args = parser.parse_args()

    app = Application()
    app.listen(8080)

    rtkclient = RtkClient(
        rtkhos=args.rtkrover_host,
        rtkport=9797,
        rtktelnetport=2023,
        event_loop=loop,
        status_callback=RtkRoverSocketHandler.new_message,
        solution_callback=RtkRoverSolutionSocketHandler.send_solution,
    )

    logger.info('rtkclient %s', rtkclient)

    loop.run_forever()


if __name__ == '__main__':
    main()
