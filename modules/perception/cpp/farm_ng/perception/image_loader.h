#ifndef FARM_NG_IMAGE_LOADER_H_
#define FARM_NG_IMAGE_LOADER_H_
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "farm_ng/core/resource.pb.h"
#include "farm_ng/perception/image.pb.h"

namespace farm_ng::perception {

class ImageLoader {
 public:
  explicit ImageLoader(bool zero_indexed = true)
      : zero_indexed_(zero_indexed) {}
  cv::Mat LoadImage(const Image& image);
  cv::Mat LoadDepthmap(const Image& image);

 private:
  void OpenVideo(const farm_ng::core::Resource& resource);
  std::unique_ptr<cv::VideoCapture> capture_;
  std::string video_name_;
  bool zero_indexed_;
};

// If the image resource has a data payload rather than a resource, this saves
// it to disk, and replaces the resource payload with the path on disk.
void ImageResourceDataToPath(Image* image);
}  // namespace farm_ng::perception

#endif
