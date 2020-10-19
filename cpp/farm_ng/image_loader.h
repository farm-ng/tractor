#ifndef FARM_NG_IMAGE_LOADER_H_
#define FARM_NG_IMAGE_LOADER_H_
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "farm_ng_proto/tractor/v1/image.pb.h"
#include "farm_ng_proto/tractor/v1/resource.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::Resource;

class ImageLoader {
 public:
  explicit ImageLoader(bool zero_indexed = true)
      : zero_indexed_(zero_indexed) {}
  cv::Mat LoadImage(const Image& image);

 private:
  void OpenVideo(const Resource& resource);
  std::unique_ptr<cv::VideoCapture> capture_;
  std::string video_name_;
  bool zero_indexed_;
};

}  // namespace farm_ng

#endif
