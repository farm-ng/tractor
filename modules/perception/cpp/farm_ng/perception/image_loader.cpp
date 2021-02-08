#include "farm_ng/perception/image_loader.h"

#include <glog/logging.h>

#include <opencv2/imgcodecs.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/pose_utils.h"

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
    LOG(WARNING) << "Could not load image: "
                 << (GetBlobstoreRoot() / image.resource().path()).string();
    frame = cv::Mat::zeros(cv::Size(image.camera_model().image_width(),
                                    image.camera_model().image_height()),
                           CV_8UC3);
  }
  CHECK(!frame.empty());
  if (frame.size().width != image.camera_model().image_width() ||
      frame.size().height != image.camera_model().image_height()) {
    LOG(FATAL) << "Image is unexpected size." << frame.size().width << "x"
               << frame.size().height << " vs "
               << image.camera_model().image_width() << "x"
               << image.camera_model().image_height();
  }
  CHECK_EQ(frame.size().width, image.camera_model().image_width());
  CHECK_EQ(frame.size().height, image.camera_model().image_height());
  return frame;
}

cv::Mat ImageLoader::LoadDepthmap(const Image& image) {
  if (!image.has_depthmap()) {
    return cv::Mat::zeros(cv::Size(image.camera_model().image_width(),
                                   image.camera_model().image_height()),
                          CV_32FC1);
  }
  cv::Mat frame;
  frame = cv::imread(
      (GetBlobstoreRoot() / image.depthmap().resource().path()).string(),
      cv::IMREAD_UNCHANGED);
  if (frame.empty()) {
    LOG(FATAL)
        << "Could not load depthmap: "
        << (GetBlobstoreRoot() / image.depthmap().resource().path()).string();
  }
  CHECK(!frame.empty());
  if (frame.size().width != image.camera_model().image_width() ||
      frame.size().height != image.camera_model().image_height()) {
    LOG(FATAL) << "Image is unexpected size." << frame.size().width << " "
               << frame.size().height;
  }
  CHECK_EQ(frame.size().width, image.camera_model().image_width());
  CHECK_EQ(frame.size().height, image.camera_model().image_height());
  if (image.depthmap().range() == Depthmap::RANGE_MM) {
    cv::Mat out;
    frame.convertTo(out, CV_32FC1, 1.0 / 1000);
    frame = out;
  } else {
    LOG(FATAL) << "Unsupported range type: "
               << Depthmap::Range_Name(image.depthmap().range());
  }
  return frame;
}

void ImageResourcePayloadToPath(core::Resource* resource,
                                const std::string& name) {
  if (resource->payload_case() != core::Resource::kData) {
    CHECK_EQ(resource->payload_case(), core::Resource::kPath)
        << resource->ShortDebugString();
    return;
  }

  std::string ext;
  if (resource->content_type() == "image/png") {
    ext = "png";
  } else if (resource->content_type() == "image/jpeg") {
    ext = "jpg";
  }
  CHECK(!ext.empty())
      << "Could not determine image extension (jpg, png supported): "
      << resource->content_type();

  auto resource_path =
      core::GetUniqueArchiveResource(name, ext, resource->content_type());
  {
    LOG(INFO) << "Writing to: " << resource_path.second.string();
    std::ofstream outf(resource_path.second.string(), std::ofstream::binary);
    CHECK(outf) << "Could not open : " << resource_path.second.string() << "\n"
                << resource_path.first.ShortDebugString();
    outf << resource->data();
  }
  CHECK(boost::filesystem::exists(resource_path.second.string()))
      << "Did not write to: " << resource_path.second.string();
  resource->CopyFrom(resource_path.first);
  CHECK_EQ(resource->payload_case(), core::Resource::kPath)
      << resource->ShortDebugString();
}

void ImageResourcePayloadToData(core::Resource* resource) {
  if (resource->payload_case() != core::Resource::kPath) {
    CHECK_EQ(resource->payload_case(), core::Resource::kData)
        << resource->ShortDebugString();
    return;
  }
  std::string bin_path = (core::GetBlobstoreRoot() / resource->path()).string();

  LOG(INFO) << "Reading " << bin_path;

  std::ifstream bin_in(bin_path, std::ifstream::binary);
  CHECK(bin_in.good());
  std::string bin_str((std::istreambuf_iterator<char>(bin_in)),
                      std::istreambuf_iterator<char>());
  resource->set_data(bin_str);
  return;
}

void ImageResourceDataToPath(Image* image) {
  uint32_t frame_number = 0;
  if (image->has_frame_number()) {
    frame_number = image->frame_number().value();
  }

  ImageResourcePayloadToPath(
      image->mutable_resource(),
      perception::FrameNameNumber(image->camera_model().frame_name(),
                                  frame_number));

  if (image->has_depthmap()) {
    ImageResourcePayloadToPath(
        image->mutable_depthmap()->mutable_resource(),
        perception::FrameNameNumber(image->camera_model().frame_name(),
                                    frame_number, "_depthmap"));
  }
}

void ImageResourcePathToData(Image* image) {
  ImageResourcePayloadToData(image->mutable_resource());

  if (image->has_depthmap()) {
    ImageResourcePayloadToData(image->mutable_depthmap()->mutable_resource());
  }
}

}  // namespace perception
}  // namespace farm_ng
