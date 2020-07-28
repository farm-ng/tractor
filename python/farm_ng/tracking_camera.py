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
import sys

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


class TrackingCamera:
    def __init__(self, name):
        self.name = name
        self.loop = asyncio.get_event_loop()
        self.pipe = rs.pipeline()
        self.running = False
        self.recording_number = 0
        self.image_data = {"left"  : None,
                           "right" : None,
                           "timestamp_ms" : None}
        
        self.pose_frame = TrackingCameraPoseFrame()

    def on_pose_frame(self, pose_frame):
        self.pose_frame = pose_frame
        event = plog.make_event({'%s/pose' % self.name: pose_frame})
        plog.writer().push(event)
        if pose_frame.frame_number % 200 == 0:
            logger.info(MessageToString(pose_frame, as_one_line=True))

    def on_image_data(self, image_data):
        self.image_data = image_data
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
            self.running= False
        
    def start_pipeline(self, record):
        self.stop_pipeline()
        cfg = rs.config()
        bag_path = None
        if record:
            self.recording_number += 1
            bag_path = os.path.join(plog.log_dir, '%03d.bag'%self.recording_number)
            logger.info('recording: %s path: %s'%(record, bag_path))
            cfg.enable_all_streams()
            cfg.enable_record_to_file(bag_path)
        else:
            cfg.enable_stream(rs.stream.pose)
            logger.info('Enabling pose stream only.')


        # Start streaming with requested config
        self.pipe.start(cfg, self.frame_callback)
        self.running = True
        

    def frame_callback(self, frame):
        try:
            self._frame_callback(frame)
        except Exception as e:
            print(e)
            
    def _frame_callback(self, frame):
        if frame.is_frameset():
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
    camera.start_pipeline(record=False)
    _ = Periodic(60, event_loop, lambda n_periods: plog.writer().flush())
    event_loop.run_forever()


if __name__ == '__main__':
    main()
