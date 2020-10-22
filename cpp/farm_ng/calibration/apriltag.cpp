#include "farm_ng/calibration/apriltag.h"


#include "glog/logging.h"

#include "farm_ng/calibration/camera_model.h"

namespace farm_ng {
std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection) {
  CHECK(detection.tag_size() > 0.0);
  double half_size = detection.tag_size() / 2.0;
  return std::array<Eigen::Vector3d, 4>(
      {Eigen::Vector3d(-half_size, -half_size, 0),
       Eigen::Vector3d(half_size, -half_size, 0.0),
       Eigen::Vector3d(half_size, half_size, 0.0),
       Eigen::Vector3d(-half_size, half_size, 0.0)});
}

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection) {
  return std::array<Eigen::Vector2d, 4>(
      {Eigen::Vector2d(detection.p(0).x(), detection.p(0).y()),
       Eigen::Vector2d(detection.p(1).x(), detection.p(1).y()),
       Eigen::Vector2d(detection.p(2).x(), detection.p(2).y()),
       Eigen::Vector2d(detection.p(3).x(), detection.p(3).y())});
}

ApriltagsFilter::ApriltagsFilter() : once_(false) {}
  void ApriltagsFilter::Reset() {
    mask_ = cv::Mat();
    once_ = false;
  }
  bool ApriltagsFilter::AddApriltags(const ApriltagDetections& detections) {
    const int n_tags = detections.detections_size();
    if (n_tags == 0) {
      Reset();
      return false;
    }

    if (mask_.empty()) {
      mask_ =
          cv::Mat::zeros(GetCvSize(detections.image().camera_model()), CV_8UC1);
    }

    cv::Mat new_mask = cv::Mat::zeros(mask_.size(), CV_8UC1);
    const int window_size = 7;
    double mean_count = 0.0;
    for (const ApriltagDetection& detection : detections.detections()) {
      for (const auto& p : detection.p()) {
        cv::Rect roi(p.x() - window_size / 2, p.y() - window_size / 2,
                     window_size, window_size);
        new_mask(roi) = mask_(roi) + 1;
        double max_val = 0.0;
        cv::minMaxLoc(new_mask(roi), nullptr, &max_val);
        mean_count += max_val / (4 * n_tags);
      }
    }
    mask_ = new_mask;
    const int kThresh = 5;
    if (mean_count > kThresh && !once_) {
      once_ = true;
      return true;
    }
    if (mean_count < kThresh) {
      once_ = false;
    }
    return false;
  }


}  // namespace farm_ng
