#include <iostream>
#include <optional>

#include <librealsense2/rs.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/intel_rs2_utils.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/rs2_bag_to_eventlog.pb.h"
#include "farm_ng/perception/image.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");
DEFINE_string(rs2_bag_path, "", "A RealSense bag file path.");
DEFINE_string(camera_frame_name, "camera01",
              "Frame name to use for the camera model.");

typedef farm_ng::core::Event EventPb;
using farm_ng::core::ArchiveProtobufAsJsonResource;
using farm_ng::core::EventBus;
using farm_ng::core::LoggingStatus;
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::Subscription;

namespace farm_ng {
namespace perception {

class Rs2BagToEventLogProgram {
 public:
  Rs2BagToEventLogProgram(EventBus& bus,
                            Rs2BagToEventLogConfiguration configuration,)
                            bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) { // The program doesn't use interactive mode at this point
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName()});

    /*
    bus_.GetEventSignal()->connect(std::bind(
        &Rs2BagToEventLogProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
    */
  }

  int run() {

    Rs2BagToEventLogResult result;

    // We prly don't need begin/end timestamps
    //result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    // Start a realsense pipeline from a recorded file to get the framesets
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_device_from_file(configuration_.bag_file_name());
    rs2::video_stream_profile profile = pipe.start(cfg);

    Image image_pb;
    image_pb.mutable_fps()->set_value(30);
    image_pb.mutable_frame_number()->set_value(0);

    LoggingStatus log = StartLogging(bus_, configuration_.name());

    int frame_count = 0;

    while (true) {

      // Fetch the next frameset (block until it comes)
      rs2::frameset frames = pipe.wait_for_frames().as<rs2::frameset>();
      ++frame_count;

      // Get the depth and color frames
      rs2::video_frame color_frame = frames.get_color_frame();
      //rs2::depth_frame depth_frame = frames.get_depth_frame();

      // Query frame size (width and height)
      const int wc = color_frame.get_width();
      const int hc = color_frame.get_height();
      //const int wd = depth_frame.get_width();
      //const int hd = depth_frame.get_height();

      // Image matrices from rs2 frames
      cv::Mat color = RS2FrameToMat(color_frame);
      //cv::Mat depth = RS2FrameToMat(depth_frame),

      if (color.empty()) {
        break;
      }

      // Display the current video frame, using case insensitive q as quit
      // signal.
      cv::imshow("Color", color);
      char c = static_cast<char>(cv::waitKey(1));
      if (tolower(c) == 'q')
      {
         cout << "Quit signal recieved, stopping video.\n\n";
         break;
      }

      std::optional<CameraModel> camera_model;
      if (!camera_model) {
        // Do we need to define a new CameraModel instance and assign it to
        // camera_model as in DefaultCameraModel()?
        camera_model.set_frame_name(camera_frame_name)
        SetCameraModelFromRs(camera_model, profile.get_intrinsics())
        image_pb.mutable_camera_model()->CopyFrom(*camera_model);
      }

      CHECK_EQ(camera_model->image_width(), w);
      CHECK_EQ(camera_model->image_height(), h);

      // Copy the data to the image protobuf, TODO
      // How do we actualy serialize the image?
      // Cannot find this in create_video_dataset.
      //image_pb.mutable_resource->CopyFrom(...)

      // Send out the image Protobuf on the event bus
      auto stamp = core::MakeTimestampNow();
      bus_.Send(
          MakeEvent(camera_model->frame_name() + "/image", image_pb, stamp));

      result.mutable_configuration()->CopyFrom(configuration_);
      result.mutable_frames()->set_path(log.recording().archive_path());

      ArchiveProtobufAsJsonResource(configuration_.name(), result);
    }

    LOG(INFO) << "Complete:\n" << status_.DebugString();

    status_.set_num_frames(frame_count);
    send_status();
    return 0;
  }

  void send_status() {
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void set_configuration(CaptureVideoDatasetConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

 private:
  EventBus& bus_;
  Rs2BagToEventLogConfiguration configuration_;
  Rs2BagToEventLogStatus status_;
};

}  // namespace perception
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::perception::Rs2BagToEventLogConfiguration config;
  config.set_name(FLAGS_name);
  config.mutable_rs2_bag_resource().set_path(FLAGS_rs2_bag_path);
  config.mutable_rs2_bag_resource().set_content_type("rs2bag");

  farm_ng::perception::Rs2BagToEventLogProgram program(bus, config,
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
