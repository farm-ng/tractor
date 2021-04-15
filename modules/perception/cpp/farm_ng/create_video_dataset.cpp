#include <iostream>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/algorithm/string.hpp>

#include "farm_ng/core/init.h"


#include "farm_ng/perception/create_video_dataset_program.h"
#include "farm_ng/perception/apriltag.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(output_config, "", "Output the config to a file.");
DEFINE_string(config, "", "Load config from a file rather than args.");

DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");
DEFINE_string(video_path, "", "A video file path.");
DEFINE_string(camera_frame_name, "camera01",
              "Frame name to use for the camera model.");

DEFINE_string(apriltag_rigs,
              "calibration_boards/board_430_506.json,"
              "calibration_boards/board_507_583.json",
              "A comma seperated list of blobstore relative paths of apriltag "
              "rigs to detect.");

DEFINE_bool(detect_apriltags, true, "Detect apriltags.");

DEFINE_int32(skip_frames, 0, "Number of frames to skip between detections.");
DEFINE_double(detection_scale, 0, "Scale the image before detection.");


int Main(farm_ng::core::EventBus& bus) {
  farm_ng::perception::CreateVideoDatasetConfiguration config;
  if (!FLAGS_config.empty()) {
    config = farm_ng::core::ReadProtobufFromJsonFile<
        farm_ng::perception::CreateVideoDatasetConfiguration>(FLAGS_config);
  } else {
    config.set_name(FLAGS_name);
    config.set_detect_apriltags(FLAGS_detect_apriltags);
    auto* video_file_camera = config.add_video_file_cameras();

    video_file_camera->mutable_video_file_resource()->set_path(
        FLAGS_video_path);
    video_file_camera->mutable_video_file_resource()->set_content_type(
        "video/mp4");
    video_file_camera->set_camera_frame_name(FLAGS_camera_frame_name);

    std::stringstream ss(FLAGS_apriltag_rigs);
    std::string token;
    while (std::getline(ss, token, ',')) {
      auto* resource = config.add_apriltag_rigs();
      boost::trim(token);
      resource->set_path(token);
      resource->set_content_type(farm_ng::core::ContentTypeProtobufJson<
                                 farm_ng::perception::ApriltagRig>());
    }
    if (FLAGS_detection_scale > 0) {
      config.mutable_detection_scale()->set_value(FLAGS_detection_scale);
    }
    if (FLAGS_skip_frames > 0) {
      config.mutable_skip_frames()->set_value(FLAGS_skip_frames);
    }
  }
  if (!FLAGS_output_config.empty()) {
    farm_ng::core::WriteProtobufToJsonFile(FLAGS_output_config, config);
    return 0;
  }

  farm_ng::perception::CreateVideoDatasetProgram program(bus, config,
                                                         FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) {}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
