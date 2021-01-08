#ifndef FARM_NG_PERCEPTION_IMAGE_SEQUENCE_WRITER_H_
#define FARM_NG_PERCEPTION_IMAGE_SEQUENCE_WRITER_H_
#include <opencv2/imgproc.hpp>

#include "farm_ng/perception/camera_model.pb.h"
#include "farm_ng/perception/video_streamer.h"

namespace farm_ng {
namespace perception {

void QuantizeDepthMap(cv::Mat depthmap, Depthmap::Range range,
                      cv::Mat* depthmap_out,
                      int output_type,  // CV_8UC1, or CV_16UC1
                      std::optional<double>* depth_near,
                      std::optional<double>* depth_far);

class ImageSequenceWriter {
 public:
  ImageSequenceWriter(const CameraModel& camera_model,
                      VideoStreamer::Mode mode);
  Image WriteImage(cv::Mat image);
  Image WriteImageWithDepth(cv::Mat image, cv::Mat depthmap,
                            VideoStreamer::DepthMode mode);

 private:
  Image image_pb_;
  std::string extension_;
  std::string content_type_;
};
}  // namespace perception
}  // namespace farm_ng
#endif
