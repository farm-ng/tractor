import asyncio
import logging
import math
import sys
import threading
import os
import farm_ng.proio_utils
from farm_ng.canbus import CANSocket
from farm_ng.joystick import MaybeJoystick
from farm_ng.motor import HubMotor
from farm_ng.periodic import Periodic
from google.protobuf.text_format import MessageToString
from farm_ng.tractor.kinematics import TractorKinematics
import numpy as np
import pyrealsense2 as rs

logger = logging.getLogger('tractor.driver')
plog = farm_ng.proio_utils.get_proio_logger()

kinematics = TractorKinematics()

rs_count = 0

def frame_callback(frame):
    global rs_count
    # print(frame, frame.is_pose_frame(), frame.is_frameset())
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        #print(left_data.shape, right_data.shape)
    # Fetch pose frame
    pose = frame.is_pose_frame()
    if pose:
        # Print some of the pose data to the terminal
        frame = frame.as_pose_frame()
        data = frame.get_pose_data()
        rs_count += 1
        if rs_count % 30 == 0:
            print(f'Frame #{frame.frame_number} Position: {data.translation} Velocity: {data.velocity}')


class TrackingCamera:
    def __init__(self):
        self.pipe = rs.pipeline()
        self.running = False
        self.recording_number = 0
        self.frame_mutex = threading.Lock()
        self.image_data = {"left"  : None,
                           "right" : None,
                           "timestamp_ms" : None}
        self.pose_data = {"translation" : None,
                          "velocity": None,
                          "rotation": None,
                          "angular_velocity": None,
                          "angular_acceleration": None,
                          "tracker_confidence": None,
                          "mapper_confidence": None,
                          "timestamp_ms": None}

    def get_pose_data(self):
        with self.frame_mutex:
            return self.pose_data

    def stop_pipeline(self):
        if self.running:
            self.pipe.stop()
            self.running= False
        
    def start_pipeline(self, record):
        self.stop_pipeline()

        cfg = rs.config()
        cfg.enable_all_streams()

        bag_path = None
        if record:
            self.recording_number += 1
            bag_path = os.path.join(plog.log_dir, '%03d.bag'%self.recording_number)
            cfg.enable_record_to_file(bag_path)

        logger.info('recording: %s path: %s'%(record, bag_path))
        # Start streaming with requested config
        self.pipe.start(cfg, self.frame_callback)
        self.running = True
        

    def frame_callback(self, frame):
        try:
            self._frame_callback(frame)
        except Exception as e:
            print(e)
            
    def _frame_callback(self, frame):
        global rs_count
        # print(frame, frame.is_pose_frame(), frame.is_frameset())
        if frame.is_frameset():
            frameset = frame.as_frameset()
            ts = frameset.get_timestamp()
            f1 = frameset.get_fisheye_frame(1).as_video_frame()
            f2 = frameset.get_fisheye_frame(2).as_video_frame()
            left_data = np.asanyarray(f1.get_data())
            right_data = np.asanyarray(f2.get_data())
            with self.frame_mutex:
                self.image_data['left'] = left_data
                self.image_data['right'] = right_data
                self.image_data['timestamp_ms'] = ts
            

            #print(left_data.shape, right_data.shape)
        # Fetch pose frame
        pose = frame.is_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            frame = frame.as_pose_frame()
            ts = frame.get_timestamp()
            data = frame.get_pose_data()
            '''
            .def_readwrite("translation", &rs2_pose::translation, "X, Y, Z values of translation, in meters (relative to initial position)")
        .def_readwrite("velocity", &rs2_pose::velocity, "X, Y, Z values of velocity, in meters/sec")
        .def_readwrite("acceleration", &rs2_pose::acceleration, "X, Y, Z values of acceleration, in meters/sec^2")
        .def_readwrite("rotation", &rs2_pose::rotation, "Qi, Qj, Qk, Qr components of rotation as represented in quaternion rotation (relative to initial position)")
        .def_readwrite("angular_velocity", &rs2_pose::angular_velocity, "X, Y, Z values of angular velocity, in radians/sec")
        .def_readwrite("angular_acceleration", &rs2_pose::angular_acceleration, "X, Y, Z values of angular acceleration, in radians/sec^2")
        .def_readwrite("tracker_confidence", &rs2_pose::tracker_confidence, "Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High")
        .def_readwrite("mapper_confidence", &rs2_pose::mapper_confidence, "Pose map confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High");
        '''
            with self.frame_mutex:
                self.pose_data['translation'] = data.translation
                self.pose_data['velocity'] = data.velocity
                self.pose_data['acceleration'] = data.acceleration
                self.pose_data['rotation'] = data.rotation
                self.pose_data['angular_velocity'] = data.angular_velocity
                self.pose_data['angular_acceleration'] = data.angular_acceleration
                self.pose_data['tracker_confidence'] = data.tracker_confidence
                self.pose_data['mapper_confidence'] = data.mapper_confidence
                self.pose_data['timestamp_ms'] = ts

        
class TractorController:
    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.command_rate_hz = 50
        self.command_period_seconds = 1.0 / self.command_rate_hz
        self.n_cycle = 0
        self.speed = 0.0
        self.angular = 0.0
        self.record_counter = 0
        self.record_toggle = False
        self.lock_out = False
        self.tracking_camera = TrackingCamera()
        self.tracking_camera.start_pipeline(record=False)
        self.can_socket = CANSocket('can0', self.event_loop)
        self.joystick = MaybeJoystick('/dev/input/js0', self.event_loop)

        radius = (17*0.0254)/2.0  # in meters, 15" diameter wheels
        gear_ratio = 29.9
        poll_pairs = 8
        self.right_motor = HubMotor(
            'right_motor',
            radius, gear_ratio, poll_pairs, 7, self.can_socket,
        )
        self.left_motor = HubMotor(
            'left_motor',
            radius, gear_ratio, poll_pairs, 9, self.can_socket,
        )

        self.control_timer = Periodic(
            self.command_period_seconds, self.event_loop,
            self._command_loop,
        )

    def _command_loop(self, n_periods):
        if (self.n_cycle % (2*self.command_rate_hz)) == 0:
            logger.info('\nright motor:\n  %s\nleft motor:\n  %s\npose:\n  %s',
                        MessageToString(self.right_motor.get_state(), as_one_line=True),
                        MessageToString(self.left_motor.get_state(), as_one_line=True),
                        self.tracking_camera.get_pose_data())

        if self.joystick.get_button_state('b', False) and self.record_counter == 0:
            self.record_counter = 200
            self.record_toggle = not self.record_toggle
            self.tracking_camera.start_pipeline(self.record_toggle)
        if self.record_counter > 0:
            self.record_counter -= 1
        
            
        self.n_cycle += 1

        brake_current=10.0
        
        if ( n_periods*self.command_period_seconds >= 0.25 or
             not self.joystick.is_connected() or self.joystick.get_axis_state('rx', -1) < 0.0):
            self.speed = 0.0
            self.angular = 0.0
            self.right_motor.send_current_brake_command(brake_current)
            self.left_motor.send_current_brake_command(brake_current)                
            self.lock_out = True
            return
        else:
            speed = np.clip(-self.joystick.get_axis_state('y', 0), -1.0, 1.0)
            angular = np.clip(-self.joystick.get_axis_state('z', 0), -1.0, 1.0)
            if self.lock_out and (np.abs(speed) > 0.1 or np.abs(angular) > 0.1):
                speed = 0.0
                angular = 0.0
            else:
                self.lock_out = False
            speed = 1.5*speed
            angular = angular*np.pi/3.0
        alpha = 0.02
        delta_speed = np.clip(speed - self.speed, -alpha, alpha)
        delta_angular = np.clip(angular - self.angular, -alpha, alpha)
        self.speed = np.clip(self.speed + delta_speed, -1.5, 1.5)
        self.angular = np.clip(self.angular + delta_angular, -np.pi/3.0, np.pi/3.0)
        left, right = kinematics.unicycle_to_wheel_velocity(self.speed, self.angular)
        self.right_motor.send_velocity_command(right)
        self.left_motor.send_velocity_command(left)


def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    controller = TractorController(event_loop)
    logger.info('Created controller %s', controller)
    _ = Periodic(60, event_loop, lambda n_periods: plog.writer().flush())
    event_loop.run_forever()


if __name__ == '__main__':
    main()
