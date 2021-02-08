#include "farm_ng/perception/video_streamer.h"

#include <gflags/gflags.h>

DEFINE_bool(jetson, false, "Use jetson hardware encoding.");

using farm_ng::core::EventBus;
using farm_ng::core::GetUniqueArchiveResource;
using farm_ng::core::MakeEvent;

namespace farm_ng {
namespace perception {

VideoStreamer::VideoStreamer(EventBus& bus, const CameraModel& camera_model,
                             Mode mode, uint64_t port)
    : bus_(bus), mode_(mode), port_(port) {
  image_pb_.mutable_camera_model()->CopyFrom(camera_model);
  image_pb_.mutable_fps()->set_value(30);
  image_pb_.mutable_frame_number()->set_value(0);
}

void VideoStreamer::ResetVideoStreamer(bool is_color) {
  // TODO(ethanrublee) look up image size from realsense profile.
  std::string gst_pipeline;
  if (mode_ == MODE_MP4_UDP) {
    // image_pb_.mutable_fps()->set_value(30);
    std::string encoder;
    if (FLAGS_jetson) {
      encoder = std::string(" omxh264enc control-rate=1 bitrate=1000000 ! ") +
                " video/x-h264, stream-format=byte-stream !";

    } else {
      encoder = std::string(
                    " x264enc bitrate=10000 speed-preset=ultrafast "
                    "tune=zerolatency ") +
                "key-int-max=15 ! video/x-h264,profile=constrained-baseline ! "
                "queue " +
                "max-size-time=100000000 ! h264parse ! ";
    }
    gst_pipeline = std::string("appsrc !") + " videoconvert ! " + encoder +
                   " rtph264pay pt=96 mtu=1400 config-interval=10 !" +
                   " udpsink port=" + std::to_string(port_);
  } else if (mode_ == MODE_MP4_FILE) {
    std::string encoder;
    if (FLAGS_jetson) {
      encoder = " omxh264enc bitrate=10000000 ! ";
    } else {
      encoder = " x264enc ! ";
    }
    auto resource_path = GetUniqueArchiveResource(
        image_pb_.camera_model().frame_name(), "mp4", "video/mp4");
    gst_pipeline =
        std::string("appsrc !") + " videoconvert ! " + encoder +
        " mp4mux ! filesink location=" + resource_path.second.string();
    image_pb_.mutable_resource()->CopyFrom(resource_path.first);
    image_pb_.mutable_frame_number()->set_value(0);

  } else {
    LOG(FATAL) << "Unsupported mode: " << mode_;
  }

  LOG(INFO) << "Running gstreamer with pipeline:\n" << gst_pipeline;
  if (mode_ == MODE_MP4_UDP) {
    LOG(INFO) << "To view streamer run:\n"
              << "gst-launch-1.0 udpsrc port=" << port_
              << " ! application/x-rtp,encoding-name=H264,payload=96 ! "
                 "rtph264depay ! h264parse ! queue ! avdec_h264 ! xvimagesink "
                 "sync=false async=false -e";
  }
  writer_ = std::make_shared<cv::VideoWriter>(
      gst_pipeline,
      0,                        // fourcc
      image_pb_.fps().value(),  // fps
      cv::Size(image_pb_.camera_model().image_width(),
               image_pb_.camera_model().image_height()),
      is_color);
}
Image VideoStreamer::AddFrame(const cv::Mat& image,
                              const google::protobuf::Timestamp& stamp) {
  if (!writer_) {
    bool is_color = image.channels() == 3;
    ResetVideoStreamer(is_color);
  }
  writer_->write(image);
  Image image_sent = image_pb_;
  if (mode_ == MODE_MP4_FILE) {
    bus_.AsyncSend(MakeEvent(image_sent.camera_model().frame_name() + "/image",
                             image_pb_, stamp));
  }

  // zero index base for the frame_number, set after send.
  image_pb_.mutable_frame_number()->set_value(image_pb_.frame_number().value() +
                                              1);

  if (mode_ == MODE_MP4_FILE) {
    if (image_pb_.frame_number().value() >= k_max_frames_) {
      writer_.reset();
    }
  }
  return image_sent;
}

void VideoStreamer::Close() {
  writer_.reset();
}

}  // namespace perception
}  // namespace farm_ng
