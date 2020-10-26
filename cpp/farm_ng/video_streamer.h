#ifndef FARM_NG_VIDEO_STREAMER_H_
#define FARM_NG_VIDEO_STREAMER_H_
#include <memory>

#include <opencv2/videoio.hpp>

#include "farm_ng/ipc.h"
#include "farm_ng_proto/tractor/v1/camera_model.pb.h"
#include "farm_ng_proto/tractor/v1/image.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::CameraModel;
using farm_ng_proto::tractor::v1::Image;

class VideoStreamer {
 public:
  enum Mode {
    MODE_UNDEFINED = 0,
    MODE_PNG_SEQUENCE = 1,
    MODE_MP4_FILE = 2,
    MODE_MP4_UDP = 3
  };

  VideoStreamer(EventBus& bus, const CameraModel& camera_model, Mode mode,
                uint64_t port = 0);

  Image AddFrame(const cv::Mat& image,
                 const google::protobuf::Timestamp& stamp);
  void Close();

 private:
  void ResetVideoStreamer(bool is_color);

  EventBus& bus_;
  Mode mode_;
  Image image_pb_;
  std::shared_ptr<cv::VideoWriter> writer_;
  const uint k_max_frames_ = 300;
  uint64_t port_;
};
}  // namespace farm_ng
#endif
