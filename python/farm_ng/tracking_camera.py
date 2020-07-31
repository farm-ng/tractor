import asyncio
import logging
import farm_ng.proio_utils
from farm_ng.periodic import Periodic
import numpy as np
import pyrealsense2 as rs
from farm_ng_proto.tractor.v1.tracking_camera_pb2 import TrackingCameraPoseFrame
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.text_format import MessageToString
import time
import os
import sys
import concurrent.futures
from subprocess import Popen, PIPE
import io

logger = logging.getLogger('tracking_camera')
plog = farm_ng.proio_utils.get_proio_logger()

def _proto_copy_from_rs_vec3(p, r):
    p.x = r.x
    p.y = r.y
    p.z = r.z

def _proto_copy_from_rs_quat(p, r):
    p.x = r.x
    p.y = r.y
    p.z = r.z
    p.w = r.w

_confidence_map = {
    0: TrackingCameraPoseFrame.CONFIDENCE_FAILED,
    1: TrackingCameraPoseFrame.CONFIDENCE_LOW,
    2: TrackingCameraPoseFrame.CONFIDENCE_MEDIUM,
    3: TrackingCameraPoseFrame.CONFIDENCE_HIGH,
}

class VideoEncoder(object):
    def __init__(self, intrinsics, out_path, framerate=30):
        print(intrinsics)
        cmd="gst-launch-1.0 fdsrc fd=0 ! videoparse width=%d height=%d framerate=10/1 format=gray8 ! videoconvert ! omxh264enc preset-level=0 bitrate=1000000 ! video/x-h264, stream-format=byte-stream ! rtph264pay mtu=1400 ! udpsink host=228.0.0.10 port=5000 sync=false async=false"%(intrinsics.width, intrinsics.height)

        # to watch this remotely run:
        # gst-launch-1.0 udpsrc multicast-group=228.0.0.10 port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! xvimagesink sync=false async=false -e
        
        print('running subprocess: %s'%cmd)
        self.converter = Popen(cmd,
            stdin=PIPE, stdout=PIPE, # stderr=io.open(os.devnull, 'wb'),
            shell=True, close_fds=True)

    def write(self, b):
        self.converter.stdin.write(b)

    def flush(self):
        self.converter.stdin.close()
        self.converter.wait()


class TrackingCamera:
    def __init__(self, name):
        self.name = name
        self.loop = asyncio.get_event_loop()
        self.pipe = rs.pipeline()
        self.encoder = None
        self.running = False
        self.image_count = 0
        self.recording_number = 0
        self.image_data = {"left"  : None,
                           "right" : None,
                           "timestamp_ms" : None}
        
        self.pose_frame = TrackingCameraPoseFrame()
        
    def on_pose_frame(self, pose_frame):
        self.pose_frame = pose_frame
        event = plog.make_event({'%s/pose' % self.name: pose_frame})
        plog.writer().push(event)
        if False and pose_frame.frame_number % 200 == 0:
            logger.info(MessageToString(pose_frame, as_one_line=True))

    def on_image_data(self, image_data):
        self.image_data = image_data
        if self.encoder is not None:
            self.encoder.write(image_data['left'])
            
            #stack = np.stack((image_data['left'], image_data['right'],image_data['right']),
            #axis=-1)
            #self.encoder.write(stack) #image_data['left'])
        # TODO(rublee) log with H264 encoding
        #event = plog.make_event({'%s/image_data' % self.name: image_data})
        #plog.writer().push(event)
        
    def get_pose_frame(self):
        return self.pose_frame

    def get_image_data(self):
        return self.image_data

    def stop_pipeline(self):
        if self.running:
            self.pipe.stop()
            if self.encoder is not None:
                self.encoder.flush()
                self.encoder = None
            self.running= False

    def start_pipeline(self, record):
        self.stop_pipeline()
        cfg = rs.config()
        bag_path = None

        cfg.enable_all_streams()
        #cfg.enable_stream(rs.stream.pose)                    

        profile = cfg.resolve(self.pipe)

        
        streams = {"left"  : profile.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
                     "right" : profile.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
        intrinsics = {"left"  : streams["left"].get_intrinsics(),
                        "right" : streams["right"].get_intrinsics()}
        print(intrinsics)
        if record:
             self.recording_number += 1
             video_path = os.path.join(plog.log_dir, 'left_%03d.mp4'%self.recording_number)
             self.encoder = VideoEncoder(intrinsics['left'], video_path)

        dev = profile.get_device()
        tm2 = dev.as_tm2()
        pose_sensor = tm2.first_pose_sensor()
        pose_jumping = pose_sensor.get_option(rs.option.enable_pose_jumping)
        reloc = pose_sensor.get_option(rs.option.enable_relocalization)
        print('pose jumping: %s'%pose_jumping)
        print('relocalization: %s'%reloc)
        pose_sensor.set_option(rs.option.enable_pose_jumping, False)
        pose_sensor.set_option(rs.option.enable_relocalization, False)
        pose_jumping = pose_sensor.get_option(rs.option.enable_pose_jumping)
        reloc = pose_sensor.get_option(rs.option.enable_relocalization)
        print('pose jumping: %s'%pose_jumping)
        print('relocalization: %s'%reloc)

        # if record:
        #     logger.info('recording: %s path: %s'%(record, bag_path))
        #     #cfg.enable_stream(rs.stream.pose)
        #     #cfg.enable_all_streams() # too much data
        #     cfg.enable_record_to_file(bag_path)
        # else:
        #     #cfg.enable_stream(rs.stream.pose)
        #     logger.info('Enabling pose stream only.')

        # Start streaming with requested config
        self.pipe.start(cfg, self.frame_callback)

        # setting options for slam:
        # https://github.com/IntelRealSense/librealsense/issues/1011
        # and what to set:
        # https://github.com/IntelRealSense/realsense-ros/issues/779 "
        # I would suggest leaving mapping enabled, but disabling
        # relocalization and jumping. This may avoid the conflict with
        # RTabMap while still giving good results."
        #
        # https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.option.html#pyrealsense2.option
        #
        self.running = True
        

    def frame_callback(self, frame):
        try:
            self._frame_callback(frame)
        except Exception as e:
            print(e)
            
    def _frame_callback(self, frame):
        if frame.is_frameset():
            self.image_count = (self.image_count  + 1)% 3
            if self.image_count == 0:
                frameset = frame.as_frameset()
                ts = frameset.get_timestamp()
                f1 = frameset.get_fisheye_frame(1).as_video_frame()
                f2 = frameset.get_fisheye_frame(2).as_video_frame()
                left_data = np.asanyarray(f1.get_data())
                right_data = np.asanyarray(f2.get_data())
                image_data = dict(left=left_data, right=right_data, timestamp_ms=ts)
                self.loop.call_soon_threadsafe(self.on_image_data, image_data)
                
        # Fetch pose frame
        pose = frame.is_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            frame = frame.as_pose_frame()
            if frame.frame_number % 4 != 0:
                return
            data = frame.get_pose_data()
            pose_frame = TrackingCameraPoseFrame()
            pose_frame.frame_number = frame.frame_number
            pose_frame.stamp_pose.FromMilliseconds(int(frame.get_timestamp()))
            _proto_copy_from_rs_vec3(pose_frame.start_pose_current.position, data.translation)
            _proto_copy_from_rs_quat(pose_frame.start_pose_current.rotation, data.rotation)
            _proto_copy_from_rs_vec3(pose_frame.velocity, data.velocity)
            _proto_copy_from_rs_vec3(pose_frame.acceleration, data.acceleration)
            _proto_copy_from_rs_vec3(pose_frame.angular_velocity, data.angular_velocity)
            _proto_copy_from_rs_vec3(pose_frame.angular_acceleration, data.angular_acceleration)
            pose_frame.tracker_confidence = _confidence_map[data.tracker_confidence]
            pose_frame.mapper_confidence = _confidence_map[data.mapper_confidence]
            self.loop.call_soon_threadsafe(self.on_pose_frame, pose_frame)

        

def main():
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)
    event_loop = asyncio.get_event_loop()
    camera = TrackingCamera('tracking_camera/front')
    camera.start_pipeline(record=True)
    _ = Periodic(60, event_loop, lambda n_periods: plog.writer().flush())
    #_ = Periodic(100, event_loop, lambda n_periods: camera.start_pipeline(True))
    try:
        event_loop.run_forever()
    except KeyboardInterrupt:
        camera.stop_pipeline()



if __name__ == '__main__':
    main()
