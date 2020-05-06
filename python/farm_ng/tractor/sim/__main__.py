import asyncio
import logging
import os
from typing import Set

import numpy as np
import tornado.gen
import tornado.ioloop
import tornado.platform.asyncio
import tornado.tcpclient
import tornado.web
import tornado.websocket
from liegroups import SE3

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
            (r'/simsocket', SimSocketHandler),
            (
                r'/third_party/(.*)', tornado.web.StaticFileHandler,
                dict(path=third_party_path),
            ),
        ]

        settings = dict(
            cookie_secret='__TODO:_GENERATE_YOUR_OWN_RANDOM_VALUE_HERE__',
            template_path=os.path.join(os.path.dirname(__file__), 'templates'),
            static_path=os.path.join(os.path.dirname(__file__), 'static'),
            xsrf_cookies=True,
            debug=True,
        )
        super().__init__(handlers, **settings)


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html')


class SimSocketHandler(tornado.websocket.WebSocketHandler):
    waiters: Set[tornado.websocket.WebSocketHandler] = set()

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        SimSocketHandler.waiters.add(self)

    def on_close(self):
        SimSocketHandler.waiters.remove(self)

    @classmethod
    def send_updates(cls, status_msg):
        logger.debug('sending message to %d waiters', len(cls.waiters))
        for waiter in cls.waiters:
            try:
                waiter.write_message(status_msg)
            except RuntimeError as e:
                logger.error('Error sending message, %s', e, exc_info=True)


class TractorKinematics:
    def __init__(self):
        self.wheel_radius = 0.0254*15/2.0
        self.wheel_base_line = 0.0254*46

    def wheel_velocity_to_unicycle(self, v_left, v_right):
        v = (self.wheel_radius/2.0)*(v_left+v_right)
        w = (self.wheel_radius/self.wheel_base_line)*(v_right-v_left)
        return v, w

    def unicycle_to_wheel_velocity(self, v, w):
        v_right = (2*v + w*self.wheel_base_line)/(2*self.wheel_radius)
        v_left = (2*v - w*self.wheel_base_line)/(2*self.wheel_radius)
        return v_left, v_right

    def evolve_world_pose_tractor(
        self, world_pose_tractor, v_left,
        v_right, delta_t,
    ):
        v, w = self.wheel_velocity_to_unicycle(v_left, v_right)
        tractor_pose_t = SE3.exp([v*delta_t, 0, 0, 0, 0, w*delta_t])
        return world_pose_tractor.dot(tractor_pose_t)


class TractorMoveToGoalController:
    def __init__(self, tractor_model):
        self.tractor_model = tractor_model
        self.K_w = 20
        self.K_v = 1.0
        self.new_goal = True

    def set_goal(self, world_pose_tractor):
        self.world_pose_tractor_goal = world_pose_tractor
        self.new_goal = True

    def update(self, world_pose_tractor_est):
        tractor_pose_goal = world_pose_tractor_est.inv().dot(
            self.world_pose_tractor_goal,
        )
        distance = np.linalg.norm(tractor_pose_goal.trans)
        if distance < 0.05:
            return 0, 0

        trans_g = tractor_pose_goal.trans
        heading_error = np.arctan2(trans_g[1], trans_g[0])
        w = np.clip(self.K_w * heading_error, -np.pi/4, np.pi/4)
        v = np.clip(self.K_v*distance, 0.0, 1.0)
        if self.new_goal:
            if np.abs(heading_error) > np.pi/64:
                v = 0  # turn in place to goal
            else:
                self.new_goal = False
        return self.tractor_model.unicycle_to_wheel_velocity(v, w)


class TractorSimulator:
    def __init__(self):
        self.model = TractorKinematics()
        self.controller = TractorMoveToGoalController(self.model)
        self.v_left = 0.0
        self.v_right = 0.0
        self.world_pose_tractor = SE3.identity()
        self.world_pose_tractor_goal = SE3.identity()

    async def loop(self):
        dt = 0.01
        t = 0.0
        count = 0
        while True:

            if np.linalg.norm(
                    self.world_pose_tractor.inv().dot(
                        self.world_pose_tractor_goal,
                    ).trans,
            ) < 0.05:
                new_goal = np.random.uniform(
                    [-10, -10], [10, 10],
                )
                self.world_pose_tractor_goal.trans[:2] = new_goal

            if count % 10 == 0:
                self.controller.set_goal(self.world_pose_tractor_goal)
                self.v_left, self.v_right = self.controller.update(
                    self.world_pose_tractor,
                )

            self.v_left += np.random.uniform(0.0, 0.5)
            self.v_right += np.random.uniform(0.0, 0.5)

            SimSocketHandler.send_updates(
                dict(
                    world_translation_tractor=self.world_pose_tractor.trans.tolist(),
                    world_quaternion_tractor=self.world_pose_tractor.rot.to_quaternion(
                        ordering='xyzw',
                    ).tolist(),
                    t=t,
                ),
            )
            t += dt
            self.world_pose_tractor = self.model.evolve_world_pose_tractor(
                self.world_pose_tractor, self.v_left, self.v_right, dt,
            )
            count += 1
            await asyncio.sleep(dt)


def main():
    tornado.platform.asyncio.AsyncIOMainLoop().install()
    ioloop = asyncio.get_event_loop()

    app = Application()
    app.listen(8989)
    sim = TractorSimulator()
    ioloop.create_task(sim.loop())
    ioloop.run_forever()


if __name__ == '__main__':
    main()
