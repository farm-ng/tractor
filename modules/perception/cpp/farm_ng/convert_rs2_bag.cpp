/* This program takes the path of an RS2 rosbag file, relative to the blobstore
 * root, and outputs an mp4 file and an event log with the following folder
 * structure:
 * <Blobstore root>
 *   - logs/<name>
 *     - <camera_frame_name>/
 *       - Color images: color-<process ID>-<counter>.mp4
 *       - Depth images: depth-<process ID>-<counter>.mp4
 *       - The program outputs mp4 files in 300-frame
 *          chunks, so there may be multiple of these
 *          files for each program run.
 *     - events-<PID>-<counter>.log
 *       - binary log of image metadata
 *     - <name>-<PID>-<counter>.json
 *       - human-readable summary of the program execution
 *
 * TODO (collinbrake | ethanruble | isherman):
 *   - normalize depth values with far/near, populate in Depthmap
 *   - output jpeg sequence for color and png sequence for depth
 *   - support interactive mode with the browser
 */

#include <iostream>
#include <optional>

#include <google/protobuf/util/time_util.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/convert_rs2_bag.pb.h"
#include "farm_ng/perception/image.pb.h"
#include "farm_ng/perception/intel_rs2_utils.h"
#include "farm_ng/perception/video_streamer.h"

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

class ConvertRS2BagProgram {
 public:
  ConvertRS2BagProgram(EventBus& bus, ConvertRS2BagConfiguration configuration,
                       bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.GetEventSignal()->connect(std::bind(&ConvertRS2BagProgram::on_event,
                                             this, std::placeholders::_1));
    bus_.AddSubscriptions({bus_.GetName(), "logger/command", "logger/status"});
  }

  int run() {
    // Get necessary config from event bus
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {"ipc_logger"});

    ConvertRS2BagResult result;

    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    // Start a realsense pipeline from a recorded file to get
    // the framesets
    rs2::pipeline pipe;
    rs2::config cfg;

    bool repeat_playback = false;
    cfg.enable_device_from_file(
        (farm_ng::core::GetBlobstoreRoot() / configuration_.rs2_bag().path())
            .string(),
        repeat_playback);

    rs2::pipeline_profile selection = pipe.start(cfg);
    auto color_stream =
        selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    CameraModel camera_model = DefaultDepthD435Model();
    camera_model.set_frame_name(configuration_.camera_frame_name());
    SetCameraModelFromRs(&camera_model, color_stream.get_intrinsics());

    Image image_pb;  // result must have access to this
                     // outside of the loop

    VideoStreamer streamer =
        VideoStreamer(bus_, camera_model,

                      // TODO(collinbrake): Support MODE_JPG_SEQUENCE
                      VideoStreamer::MODE_MP4_FILE);

    LoggingStatus log = StartLogging(bus_, configuration_.name());

    while (true) {
      try {
        // Fetch the next frameset (block until it comes)
        rs2::frameset frames = pipe.wait_for_frames().as<rs2::frameset>();

        // Get the depth and color frames
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // Query frame size (width and height)
        const int wc = color_frame.get_width();
        const int hc = color_frame.get_height();
        const int wd = depth_frame.get_width();
        const int hd = depth_frame.get_height();

        CHECK_EQ(camera_model.image_width(), wc);
        CHECK_EQ(camera_model.image_height(), hc);

        // Image matrices from rs2 frames
        cv::Mat color = RS2FrameToMat(color_frame);
        cv::Mat depth;
        RS2FrameToMat(depth_frame).convertTo(depth, CV_8UC1);

        if (color.empty()) {
          break;
        }

        auto frame_stamp =
            google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
                color_frame.get_timestamp());
        image_pb = streamer.AddFrameWithDepthmap(color, depth, frame_stamp);

        // Send out the image Protobuf on the event bus
        auto stamp = core::MakeTimestampNow();
        bus_.Send(
            MakeEvent(camera_model.frame_name() + "/image", image_pb, stamp));

        // zero index base for the frame_number, set after
        // send.
        image_pb.mutable_frame_number()->set_value(
            image_pb.frame_number().value() + 1);

        status_.set_num_frames(status_.num_frames() + 1);

      } catch (const rs2::error& e) {
        std::stringstream ss;
        ss << "RealSense error calling " << e.get_failed_function() << "("
           << e.get_failed_args() << "):\n    " << e.what();
        LOG(ERROR) << ss.str();
        break;
      }
    }

    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_dataset()->set_path(log.recording().archive_path());
    result.mutable_dataset()->set_content_type(
        "application/farm_ng.eventlog.v1");
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());

    ArchiveProtobufAsJsonResource(configuration_.name(), result);

    LOG(INFO) << "Complete:\n" << status_.DebugString();

    send_status();
    return 0;
  }

  void send_status() {
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  bool on_configuration(const EventPb& event) {
    ConvertRS2BagConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(ConvertRS2BagConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
  ConvertRS2BagConfiguration configuration_;
  ConvertRS2BagStatus status_;
};

}  // namespace perception
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::perception::ConvertRS2BagConfiguration config;
  config.set_name(FLAGS_name);
  config.set_camera_frame_name(FLAGS_camera_frame_name);
  config.mutable_rs2_bag()->set_path(FLAGS_rs2_bag_path);
  config.mutable_rs2_bag()->set_content_type("application/x-rosbag");

  farm_ng::perception::ConvertRS2BagProgram program(bus, config,
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
