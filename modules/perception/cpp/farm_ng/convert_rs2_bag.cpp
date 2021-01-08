/* This program takes the path of an RS2 rosbag file, relative to the blobstore
 * root, and outputs an mp4 file and an event log with the following folder
 * structure:
 * <Blobstore root>
 *   - logs/<name>
 *     - <camera_frame_name>/
 *       - Color images: <process ID (PID)>-<counter>.jpg
 *       - Depth images: depth-<PID>-<counter>.png/jpg
 *     - events-<PID from IPC logger>-<counter>.log
 *       - binary log of image metadata
 *     - <name>-<PID>-<counter>.json
 *       - human-readable summary of the program execution
 *
 * TODO (collinbrake | ethanruble | isherman):
 *   - check for invalid (0) depth values in depth quantization
 *   - support interactive mode with the browser
 */

#include <iostream>
#include <optional>

#include <google/protobuf/util/time_util.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/convert_rs2_bag.pb.h"
#include "farm_ng/perception/image.pb.h"
#include "farm_ng/perception/intel_rs2_utils.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/video_streamer.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "", "a dataset name, used in the output archive name");
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

void QuantizeDepthMap(cv::Mat depthmap, Depthmap::Range range,
                      cv::Mat* depthmap_out,
                      int output_type,  // CV_8UC1, or CV_16UC1
                      std::optional<double>* depth_near,
                      std::optional<double>* depth_far) {
  if (range == Depthmap::RANGE_MM) {
    // for RANGE_MM we don't compute the near and far depth.
    depth_near->reset();
    depth_far->reset();
    // if 16 bit, and mm, assume 16bit is already in mm.
    if (depthmap.type() == CV_16UC1) {
      *depthmap_out = depthmap;
    } else if (depthmap.type() == CV_32FC1) {
      CHECK_EQ(output_type, CV_16UC1) << "Only 16bit makes sense for mm";
      // if 32 bit float, assume depth in meters, so convert to mm.
      depthmap.convertTo(*depthmap_out, CV_16UC1, 1000);
    } else {
      CHECK(depthmap.type() == CV_32FC1 || depthmap.type() == CV_16UC1);
    }
    return;
  }
  double min_val, max_val;

  cv::Mat depthmap_float;
  depthmap.convertTo(depthmap_float, CV_32F);
  cv::minMaxLoc(depthmap_float, &min_val, &max_val);

  double depth_scale = 1.0;
  if (depthmap.type() == CV_16UC1) {
    depth_scale = 0.001;
  }
  // depth_far and depth_near are in meters.
  *depth_far = max_val * depth_scale;
  *depth_near = min_val * depth_scale;

  // depth_normalized will be between [0,1].
  // See depthmap.proto for details.
  cv::Mat depth_normalized;
  if (range == Depthmap::RANGE_INVERSE) {
    depth_normalized =
        ((max_val - min_val) / (depthmap_float - min_val + max_val - min_val) -
         0.5) /
        0.5;
  } else if (range == Depthmap::RANGE_LINEAR) {
    depth_normalized = (depthmap_float - min_val) / (max_val - min_val);
  }
  if (output_type == CV_8UC1) {
    depth_normalized.convertTo(*depthmap_out, CV_8UC1,
                               std::numeric_limits<uint8_t>::max());
  } else if (output_type == CV_16UC1) {
    depth_normalized.convertTo(*depthmap_out, CV_16UC1,
                               std::numeric_limits<uint16_t>::max());
  } else {
    LOG(FATAL) << "output_type must be CV_8UC1 or CV_16UC1";
  }
}

class ImageSequenceWriter {
 public:
  ImageSequenceWriter(const CameraModel& camera_model,
                      VideoStreamer::Mode mode) {
    image_pb_.mutable_camera_model()->CopyFrom(camera_model);
    image_pb_.mutable_frame_number()->set_value(0);
    CHECK(mode == VideoStreamer::MODE_JPG_SEQUENCE ||
          mode == VideoStreamer::MODE_PNG_SEQUENCE);
    if (mode == VideoStreamer::MODE_JPG_SEQUENCE) {
      extension_ = "jpg";
      content_type_ = "image/jpeg";
    }
    if (mode == VideoStreamer::MODE_PNG_SEQUENCE) {
      extension_ = "png";
      content_type_ = "image/png";
    }
  }
  Image WriteImage(cv::Mat image) {
    // copy image_pb_ field, because we're going
    // to increment frame_number for the next image,
    // but want to return the current frame_number.
    Image image_pb(image_pb_);
    image_pb_.mutable_frame_number()->set_value(
        image_pb_.frame_number().value() + 1);

    auto resource_path = core::GetUniqueArchiveResource(
        FrameNameNumber(image_pb.camera_model().frame_name(),
                        image_pb.frame_number().value()),
        extension_, content_type_);

    image_pb.mutable_resource()->CopyFrom(resource_path.first);
    // LOG(INFO) << resource_path.second.string();
    CHECK(cv::imwrite(resource_path.second.string(), image))
        << "Could not write: " << resource_path.second;
    return image_pb;
  }

  Image WriteImageWithDepth(cv::Mat image, cv::Mat depthmap,
                            VideoStreamer::DepthMode mode) {
    CHECK_EQ(image.size().width, depthmap.size().width);
    CHECK_EQ(image.size().height, depthmap.size().height);
    Image image_pb = WriteImage(image);
    cv::Mat depthmap_q;
    std::optional<double> depth_near, depth_far;
    Depthmap::Range range;
    int output_type;
    std::string depth_extension;
    std::string depth_content_type;
    if (mode == VideoStreamer::DEPTH_MODE_LINEAR_16BIT_PNG) {
      range = Depthmap::RANGE_LINEAR;
      output_type = CV_16UC1;
      depth_extension = "png";
      depth_content_type = "image/png";
    } else if (mode == VideoStreamer::DEPTH_MODE_INVERSE_16BIT_PNG) {
      range = Depthmap::RANGE_INVERSE;
      output_type = CV_16UC1;
      depth_extension = "png";
      depth_content_type = "image/png";

    } else if (mode == VideoStreamer::DEPTH_MODE_INVERSE_8BIT_JPG) {
      range = Depthmap::RANGE_INVERSE;
      output_type = CV_8UC1;
      depth_extension = "jpg";
      depth_content_type = "image/jpeg";

    } else {
      LOG(FATAL) << "unsupported mode: " << mode;
    }
    QuantizeDepthMap(depthmap, range, &depthmap_q, output_type, &depth_near,
                     &depth_far);
    CHECK(depth_near);
    CHECK(depth_far);
    LOG(INFO) << "Depth near : " << *depth_near
              << " Depth far : " << *depth_far;
    auto resource_path = core::GetUniqueArchiveResource(
        FrameNameNumber(image_pb.camera_model().frame_name(),
                        image_pb.frame_number().value(), "_depthmap"),
        depth_extension, depth_content_type);

    // LOG(INFO) << resource_path.second.string();
    CHECK(cv::imwrite(resource_path.second.string(), depthmap_q))
        << "Could not write depthmap: " << resource_path.second;
    image_pb.mutable_depthmap()->set_range(range);

    image_pb.mutable_depthmap()->mutable_depth_near()->set_value(*depth_near);
    image_pb.mutable_depthmap()->mutable_depth_far()->set_value(*depth_far);
    image_pb.mutable_resource()->CopyFrom(resource_path.first);
    return image_pb;
  }
  Image image_pb_;
  std::string extension_;
  std::string content_type_;
};
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
    bus_.AddSubscriptions({bus_.GetName()});
  }

  int run() {
    // Get necessary config from event bus
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    ConvertRS2BagResult result;

    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    // Start a realsense pipeline from a recorded file to get
    // the framesets
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color(RS2_STREAM_COLOR);

    bool repeat_playback = false;
    cfg.enable_device_from_file(
        (farm_ng::core::GetBlobstoreRoot() / configuration_.rs2_bag().path())
            .string(),
        repeat_playback);
    rs2::pipeline_profile profile = pipe.start(cfg);

    // Get depth scale to convert to mm
    float depth_scale = profile.get_device()
                            .query_sensors()
                            .front()
                            .as<rs2::depth_sensor>()
                            .get_depth_scale();

    // Get intrinsics for camera model
    auto color_stream =
        profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    CameraModel camera_model;
    camera_model.set_frame_name(configuration_.camera_frame_name());
    SetCameraModelFromRs(&camera_model, color_stream.get_intrinsics());

    Image image_pb;  // result must have access to this outside of the loop

    ImageSequenceWriter writer(camera_model, VideoStreamer::MODE_JPG_SEQUENCE);

    std::string log_path = (core::GetBucketRelativePath(core::BUCKET_LOGS) /
                            boost::filesystem::path(configuration_.name()))
                               .string();

    core::SetArchivePath(log_path);

    auto resource_path = farm_ng::core::GetUniqueArchiveResource(
        "events", "log", "application/farm_ng.eventlog.v1");

    core::EventLogWriter log_writer(resource_path.second);

    while (true) {
      try {
        // Fetch the next frameset (block until it comes)
        rs2::frameset frames = pipe.wait_for_frames().as<rs2::frameset>();

        // Align frameset to the color frame so that
        // depth map is same size as color frame
        frames = align_to_color.process(frames);

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
        CHECK_EQ(camera_model.image_width(), wd);
        CHECK_EQ(camera_model.image_height(), hd);

        // Image matrices from rs2 frames
        cv::Mat color = RS2FrameToMat(color_frame);
        cv::Mat depthmap_mm = RS2FrameToMat(depth_frame) * depth_scale * 1000;

        if (color.empty()) {
          break;
        }

        auto frame_stamp =
            google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
                color_frame.get_timestamp());

        image_pb = writer.WriteImageWithDepth(
            color, depthmap_mm, VideoStreamer::DEPTH_MODE_INVERSE_8BIT_JPG);

        // Write image_pb to the event log.
        log_writer.Write(MakeEvent(camera_model.frame_name() + "/image",
                                   image_pb, frame_stamp));

        status_.set_num_frames(image_pb.frame_number().value());

      } catch (const rs2::error& e) {
        std::stringstream ss;
        ss << "RealSense error calling " << e.get_failed_function() << "("
           << e.get_failed_args() << "):\n    " << e.what();
        LOG(ERROR) << ss.str();
        break;
      }
    }

    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_dataset()->CopyFrom(resource_path.first);
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
  std::string dataset_name = FLAGS_name;
  CHECK(boost::filesystem::exists(farm_ng::core::GetBlobstoreRoot() /
                                  FLAGS_rs2_bag_path))
      << "Invalid file name.";
  if (dataset_name == "") {
    dataset_name =
        boost::filesystem::change_extension(FLAGS_rs2_bag_path, "").string();
  }
  config.set_name(dataset_name);
  config.set_camera_frame_name(FLAGS_camera_frame_name);
  config.mutable_rs2_bag()->set_path(FLAGS_rs2_bag_path);
  config.mutable_rs2_bag()->set_content_type("application/x-rosbag");

  farm_ng::perception::ConvertRS2BagProgram program(bus, config,
                                                    FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) {}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
