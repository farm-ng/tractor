// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>

#include <farm_ng/ipc.h>
#include <farm_ng/sophus_protobuf.h>

#include <farm_ng_proto/tractor/v1/geometry.pb.h>
#include <farm_ng_proto/tractor/v1/tracking_camera.pb.h>
#include <google/protobuf/util/time_util.h>

#include <opencv2/opencv.hpp>

using farm_ng_proto::tractor::v1::Event;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::TrackingCameraPoseFrame;

namespace farm_ng {

// Convert rs2::frame to cv::Mat
// https://raw.githubusercontent.com/IntelRealSense/librealsense/master/wrappers/opencv/cv-helpers.hpp
static cv::Mat frame_to_mat(const rs2::frame& f) {
  using namespace cv;
  using namespace rs2;

  auto vf = f.as<video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();
  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    Mat r_bgr;
    cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
    return r_bgr;
  } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
    return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
    return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
  }

  throw std::runtime_error("Frame format is not supported yet!");
}
void SetVec3FromRs(farm_ng_proto::tractor::v1::Vec3* out, rs2_vector vec) {
  out->set_x(vec.x);
  out->set_y(vec.y);
  out->set_z(vec.z);
}
void SetQuatFromRs(farm_ng_proto::tractor::v1::Quaternion* out,
                   rs2_quaternion vec) {
  out->set_w(vec.w);
  out->set_x(vec.x);
  out->set_y(vec.y);
  out->set_z(vec.z);
}

TrackingCameraPoseFrame::Confidence ToConfidence(int x) {
  switch (x) {
    case 0:
      return TrackingCameraPoseFrame::CONFIDENCE_FAILED;
    case 1:
      return TrackingCameraPoseFrame::CONFIDENCE_LOW;
    case 2:
      return TrackingCameraPoseFrame::CONFIDENCE_MEDIUM;
    case 3:
      return TrackingCameraPoseFrame::CONFIDENCE_HIGH;
    default:
      return TrackingCameraPoseFrame::CONFIDENCE_UNSPECIFIED;
  }
}

TrackingCameraPoseFrame ToPoseFrame(const rs2::pose_frame& rs_pose_frame) {
  TrackingCameraPoseFrame pose_frame;
  pose_frame.set_frame_number(rs_pose_frame.get_frame_number());
  *pose_frame.mutable_stamp_pose() =
      google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          rs_pose_frame.get_timestamp());
  auto pose_data = rs_pose_frame.get_pose_data();
  SetVec3FromRs(pose_frame.mutable_start_pose_current()->mutable_position(),
                pose_data.translation);
  SetQuatFromRs(pose_frame.mutable_start_pose_current()->mutable_rotation(),
                pose_data.rotation);
  SetVec3FromRs(pose_frame.mutable_velocity(), pose_data.velocity);
  SetVec3FromRs(pose_frame.mutable_acceleration(), pose_data.acceleration);
  SetVec3FromRs(pose_frame.mutable_angular_velocity(),
                pose_data.angular_velocity);
  SetVec3FromRs(pose_frame.mutable_angular_acceleration(),
                pose_data.angular_acceleration);
  pose_frame.set_tracker_confidence(ToConfidence(pose_data.tracker_confidence));
  pose_frame.set_mapper_confidence(ToConfidence(pose_data.mapper_confidence));
  return pose_frame;
}

Event ToNamedPoseEvent(const rs2::pose_frame& rs_pose_frame) {
  // TODO(ethanrublee) support front and rear cameras.
  NamedSE3Pose vodom_pose_t265;
  // here we distinguish where visual_odom frame by which camera it refers to,
  // will have to connect each camera pose to the mobile base with an extrinsic
  // transform
  vodom_pose_t265.set_frame_b("odometry/tracking_camera/front");
  vodom_pose_t265.set_frame_a("tracking_camera/front");
  auto pose_data = rs_pose_frame.get_pose_data();
  SetVec3FromRs(vodom_pose_t265.mutable_a_pose_b()->mutable_position(),
                pose_data.translation);
  SetQuatFromRs(vodom_pose_t265.mutable_a_pose_b()->mutable_rotation(),
                pose_data.rotation);

  Sophus::SE3d se3;
  ProtoToSophus(vodom_pose_t265.a_pose_b(), &se3);
  SophusToProto(se3.inverse(), vodom_pose_t265.mutable_a_pose_b());

  Event event =
      farm_ng::MakeEvent("pose/tracking_camera/front", vodom_pose_t265);
  *event.mutable_stamp() =
      google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          rs_pose_frame.get_timestamp());
  return event;
}

class TrackingCameraClient {
 public:
  TrackingCameraClient(boost::asio::io_service& io_service)
      : io_service_(io_service), event_bus_(GetEventBus(io_service_)) {
    // TODO(ethanrublee) look up image size from realsense profile.
    std::string cmd0 =
        std::string("appsrc !") + " videoconvert ! " +
        //" x264enc bitrate=600 speed-preset=ultrafast tune=zerolatency
        // key-int-max=15 ! video/x-h264,profile=constrained-baseline ! queue
        // max-size-time=100000000 ! h264parse ! "
        " omxh264enc control-rate=1 bitrate=1000000 ! " +
        " video/x-h264, stream-format=byte-stream !" +
        " rtph264pay pt=96 mtu=1400 config-interval=10 !" +
        " udpsink host=239.20.20.20 auto-multicast=true  port=5000";
    std::cerr << "Running gstreamer with pipeline:\n" << cmd0 << std::endl;
    std::cerr << "To view streamer run:\n"
              << "gst-launch-1.0 udpsrc multicast-group=239.20.20.20 port=5000 "
                 "! application/x-rtp,encoding-name=H264,payload=96 ! "
                 "rtph264depay ! h264parse ! queue ! avdec_h264 ! xvimagesink "
                 "sync=false async=false -e"
              << std::endl;

    writer_.reset(new cv::VideoWriter(cmd0,
                                      0,   // fourcc
                                      10,  // fps
                                      cv::Size(848, 800),
                                      false  // isColor
                                      ));

    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    auto profile = cfg.resolve(pipe_);
    auto tm2 = profile.get_device().as<rs2::tm2>();
    auto pose_sensor = tm2.first<rs2::pose_sensor>();

    // setting options for slam:
    // https://github.com/IntelRealSense/librealsense/issues/1011
    // and what to set:
    //  https://github.com/IntelRealSense/realsense-ros/issues/779 "
    // I would suggest leaving mapping enabled, but disabling
    // relocalization and jumping. This may avoid the conflict with
    // RTabMap while still giving good results."

    pose_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 0);
    pose_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 0);

    // Start pipeline with chosen configuration
    pipe_.start(cfg, std::bind(&TrackingCameraClient::frame_callback, this,
                               std::placeholders::_1));
  }

  // The callback is executed on a sensor thread and can be called
  // simultaneously from multiple sensors Therefore any modification to common
  // memory should be done under lock
  void frame_callback(const rs2::frame& frame) {
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      rs2::video_frame fisheye_frame = fs.get_fisheye_frame(0);
      cv::Mat frame_0 = frame_to_mat(fisheye_frame);
      std::lock_guard<std::mutex> lock(mtx_);
      count_ = (count_ + 1) % 3;
      if (count_ == 0) {
        writer_->write(frame_0);
      }
    } else if (rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>()) {
      auto pose_data = pose_frame.get_pose_data();
      // Print the x, y, z values of the translation, relative to initial
      // position
      // std::cout << "\r Device Position: " << std::setprecision(3) <<
      // std::fixed << pose_data.translation.x << " " << pose_data.translation.y
      // << " " << pose_data.translation.z << " (meters)";
      event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/pose",
                                         ToPoseFrame(pose_frame)));
      event_bus_.Send(ToNamedPoseEvent(pose_frame));
    }
  }
  boost::asio::io_service& io_service_;
  EventBus& event_bus_;
  std::unique_ptr<cv::VideoWriter> writer_;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe_;
  std::mutex mtx_;
  int count_ = 0;
};
}  // namespace farm_ng

int main(int argc, char* argv[]) try {
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  boost::asio::io_service io_service;
  farm_ng::TrackingCameraClient client(io_service);
  io_service.run();
  return EXIT_SUCCESS;
} catch (const rs2::error& e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
