// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>

#include <farm_ng/ipc.h>
#include <farm_ng_proto/tractor/v1/tracking_camera.pb.h>
#include <google/protobuf/util/time_util.h>
using farm_ng_proto::tractor::v1::TrackingCameraPoseFrame;

TrackingCameraPoseFrame ToPoseFrame(const rs2::pose_frame& rs_pose_frame) {
  TrackingCameraPoseFrame pose_frame;
  pose_frame.set_frame_number(rs_pose_frame.get_frame_number());
  *pose_frame.mutable_stamp_pose() = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(rs_pose_frame.get_timestamp());
  return pose_frame;
}

// The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
// Therefore any modification to common memory should be done under lock
void frame_callback(farm_ng::EventBus* event_bus, const rs2::frame& frame)
{
  if (rs2::frameset fs = frame.as<rs2::frameset>())
    {
      // With callbacks, all synchronized stream will arrive in a single frameset
      for (const rs2::frame& f : fs) {
      }
    }
  else if(rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>())
    {
        auto pose_data = pose_frame.get_pose_data();
        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
	  pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
	event_bus->Send(farm_ng::MakeEvent("tracking_camera/front/pose", ToPoseFrame(pose_frame)));
    }
};

int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    boost::asio::io_service io_service;

    farm_ng::EventBus & event_bus = farm_ng::GetEventBus(io_service);

    // Start pipeline with chosen configuration
    pipe.start(cfg, std::bind(frame_callback, &event_bus, std::placeholders::_1));

    io_service.run();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
