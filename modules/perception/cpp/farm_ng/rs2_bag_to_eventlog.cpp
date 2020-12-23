#include <iostream>
#include <optional>

#include <librealsense2/rs.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/rs2_bag_to_eventlog.pb.h"
#include "farm_ng/perception/image.pb.h"

//DEFINE_bool(interactive, false, "receive program args via eventbus");

typedef farm_ng::core::Event EventPb;
using farm_ng::core::ArchiveProtobufAsJsonResource;
using farm_ng::core::EventBus;
using farm_ng::core::LoggingStatus;
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::Subscription;

namespace farm_ng {
namespace perception {

class VideoFromBagProgram {
 public:
  VideoFromBagProgram(EventBus& bus,
                            VideoFromBagConfiguration configuration)
                            //,bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName(), "logger/command", "logger/status"});

    bus_.GetEventSignal()->connect(std::bind(
        &VideoFromBagProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {

    // Start a realsense pipeline from a recorded file to get the framesets
    rs2::pipeline pipe;
    rs2::config cfg;

    // Do we need to go through this bag_file_camera object for this application?
    cfg.enable_device_from_file(configuration_.bag_file_camera().camera //...);
    pipe.start(cfg);

    Image image_pb;
    image_pb.mutable_fps()->set_value(30);
    image_pb.mutable_frame_number()->set_value(0);
    //image_pb.mutable_resource()->CopyFrom(
    //    configuration_.video_file_cameras(0).video_file_resource());

    int frameCount = 0;

    while (true) {

      // Fetch the next frameset (block until it comes)
      rs2::frameset frames = pipe.wait_for_frames().as<rs2::frameset>();
      ++frameCount;

      // Get the depth and color frames
      rs2::depth_frame depthFrame = frames.get_depth_frame();
      rs2::video_frame colorFrame = frames.get_color_frame();

      // Query frame size (width and height)
      const int wd = depthFrame.get_width();
      const int hd = depthFrame.get_height();
      const int wc = colorFrame.get_width();
      const int hc = colorFrame.get_height();

      // Create OpenCV matrices of size (w,h) from the data
      cv::Mat color(cv::Size(w, h), CV_8UC3, (void *)colorFrame.get_data(),
                cv::Mat::AUTO_STEP);
      cv::Mat depth(cv::Size(w, h), CV_8UC1, (void *)depthFrame.get_data(),
                cv::Mat::AUTO_STEP);

      // Display the current video frame, using case insensitive q as quit
      // signal.
      cv::imshow("Color", color);
      char c = static_cast<char>(cv::waitKey(1));
      if (tolower(c) == 'q')
      {
         cout << "Quit signal recieved, stopping video.\n\n";
         break;
      }

      if (image.empty()) {
        break;
      }

      std::optional<CameraModel> camera_model;
      if (!camera_model) {
        camera_model = DefaultCameraModel(
            configuration_.bag_file_camera().camera_frame_name(),
            w, h);
        image_pb.mutable_camera_model()->CopyFrom(*camera_model);
      }

      CHECK_EQ(camera_model->image_width(), w);
      CHECK_EQ(camera_model->image_height(), h);

      cv::Mat gray;
      if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      } else {
        CHECK_EQ(1, image.channels())
            << "Image should be grayscale or BGR, something is wrong...";
        gray = image.clone();
      }
    }

    LOG(INFO) << "Complete:\n" << status_.DebugString();
    return 0;
  }

 private:
  VideoFromBagConfiguration configuration_;
  VideoFromBagStatus status_;
};

}  // namespace perception
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::perception::VideoFromBagConfiguration config;
  config.set_name(FLAGS_name);

  farm_ng::perception::VideoFromBagProgram program(bus, config,
                                                         FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) {
  farm_ng::core::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
