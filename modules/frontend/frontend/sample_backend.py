# TODO: Make pylint handle protobuf generated code properly, possibly with pylint-protobuf
# pylint: disable=no-member
import asyncio
import logging
import os

import tornado.ioloop
import tornado.platform.asyncio
import tornado.tcpclient
import tornado.web
import tornado.websocket

logger = logging.getLogger('sample')
logger.setLevel(logging.INFO)


class Application(tornado.web.Application):
    def __init__(self):
        dist_path = os.path.join(
            os.environ['FARM_NG_ROOT'],
            'app/frontend/dist',
        )

        handlers = [
            (r'/sample', SampleHandler),
        ]

        handlers.append(
            (r'/(.*)', tornado.web.StaticFileHandler, {'path': dist_path, 'default_filename': 'index.html'}),
        )
        settings = dict(
            debug=True,
        )
        super().__init__(handlers, **settings)


class SampleHandler(tornado.web.RequestHandler):
    def get(self):
        self.write('Hello world!')


def main():
    tornado.platform.asyncio.AsyncIOMainLoop().install()
    ioloop = asyncio.get_event_loop()
    app = Application()
    app.listen(9010)
    ioloop.run_forever()


if __name__ == '__main__':
    main()
