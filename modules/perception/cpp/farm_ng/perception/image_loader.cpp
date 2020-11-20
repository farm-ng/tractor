#include "farm_ng/perception/image_loader.h"

#include <glog/logging.h>

#include <opencv2/imgcodecs.hpp>

#include "farm_ng/core/blobstore.h"

using farm_ng::core::GetBlobstoreRoot;
using farm_ng::core::Resource;

namespace farm_ng {
namespace perception {

void ImageLoader::OpenVideo(const Resource& resource) {
  CHECK_EQ(resource.content_type(), "video/mp4");

  if (!capture_) {
    video_name_ = (GetBlobstoreRoot() / resource.path()).string();
    capture_.reset(new cv::VideoCapture(video_name_));
  } else {
    if (video_name_ != (GetBlobstoreRoot() / resource.path()).string()) {
      capture_.reset(nullptr);
      OpenVideo(resource);
    }
  }
  CHECK(capture_->isOpened()) << "Video is not opened: " << video_name_;
}

cv::Mat ImageLoader::LoadImage(const Image& image) {
  cv::Mat frame;
  if (image.resource().content_type() == "video/mp4") {
    OpenVideo(image.resource());
    VLOG(2) << image.resource().path()
            << " frame number: " << image.frame_number().value();

    int frame_number = image.frame_number().value();
    if (!zero_indexed_) {
      CHECK_GT(frame_number, 0);
      frame_number -= 1;
    }
    capture_->set(cv::CAP_PROP_POS_FRAMES, frame_number);
    CHECK_EQ(frame_number, uint32_t(capture_->get(cv::CAP_PROP_POS_FRAMES)));
    *capture_ >> frame;
  } else {
    frame = cv::imread((GetBlobstoreRoot() / image.resource().path()).string(),
                       cv::IMREAD_UNCHANGED);
  }
  if (frame.empty()) {
    frame = cv::Mat::zeros(cv::Size(image.camera_model().image_width(),
                                    image.camera_model().image_height()),
                           CV_8UC3);
  }
  CHECK(!frame.empty());
  if (frame.size().width != image.camera_model().image_width() ||
      frame.size().height != image.camera_model().image_height()) {
    frame = cv::Mat::zeros(cv::Size(image.camera_model().image_width(),
                                    image.camera_model().image_height()),
                           CV_8UC3);
  }
  CHECK_EQ(frame.size().width, image.camera_model().image_width());
  CHECK_EQ(frame.size().height, image.camera_model().image_height());
  return frame;
}

}  // namespace perception
}  // namespace farm_ng
