#ifndef FARM_NG_VIDEO_STREAMER_H_
#define FARM_NG_VIDEO_STREAMER_H_
#include <memory>

#include <opencv2/videoio.hpp>

#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/camera_model.pb.h"
#include "farm_ng/perception/image.pb.h"

namespace farm_ng {
namespace perception {

class VideoStreamer {
 public:
  enum Mode {
    MODE_UNDEFINED = 0,
    MODE_PNG_SEQUENCE = 1,
    MODE_MP4_FILE = 2,
    MODE_MP4_UDP = 3
  };

  VideoStreamer(farm_ng::core::EventBus& bus, const CameraModel& camera_model,
                Mode mode, uint64_t port = 0);

  Image AddFrame(const cv::Mat& image,
                 const google::protobuf::Timestamp& stamp);
  void Close();

 private:
  void ResetVideoStreamer(bool is_color);

  farm_ng::core::EventBus& bus_;
  Mode mode_;
  Image image_pb_;
  std::shared_ptr<cv::VideoWriter> writer_;
  const uint k_max_frames_ = 300;
  uint64_t port_;
};

}  // namespace perception
}  // namespace farm_ng
#endif
