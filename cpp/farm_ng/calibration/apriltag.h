#ifndef FARM_NG_CALIBRATION_APRILTAG_H_
#define FARM_NG_CALIBRATION_APRILTAG_H_
#include <array>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/camera_model.pb.h"

namespace farm_ng {

using farm_ng_proto::tractor::v1::ApriltagConfig;
using farm_ng_proto::tractor::v1::ApriltagDetection;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::CameraModel;
using farm_ng_proto::tractor::v1::TagLibrary;

class EventBus;

std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection);

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection);

double TagSize(const TagLibrary& tag_library, int tag_id);

// This class is meant to help filter apriltags, returning true once after the
// camera becomes relatively stationary.  To allow for a capture program which
// automatically captures a sparse set of unique view points for calibration
// after person or robot moves to position and stops for some reasonable period
// and then continues. It uses a simple 2d accumulated mask based hueristic.
//
//   1. for each detected apriltag point, define a small ROI (e.g. 7x7)
//       1. Construct a new mask the size of the detection image,
//          which is set to 0 everywhere except for in the ROI
//            mask = 0
//            mask(ROI) = previous_mask(ROI) + 1
//       2. Find the max count in the ROI, this is essentially the number of
//       frames where this tag point has kept roughly within the ROI.
//   2. Compute the mean of the max counts.
//   3. If the mean is moves above a threshold, determined experimentally to
//   mean stable, 5 at my desk lab, then this detection is considered stable. If
//   the camera stays still, we're only interested in the transition, because we
//   don't want duplicate frames.
//
//   NOTE If the camera moves slowly but constantly, this tends to capture every
//   other frame.
class ApriltagsFilter {
 public:
  ApriltagsFilter();
  void Reset();
  bool AddApriltags(const ApriltagDetections& detections);

 private:
  cv::Mat mask_;
  bool once_;
};
class ApriltagDetector {
 public:
  ApriltagDetector(const CameraModel& camera_model,
                   EventBus* event_bus = nullptr);

  ~ApriltagDetector();

  void Close();

  ApriltagDetections Detect(const cv::Mat& gray,
                            const google::protobuf::Timestamp& stamp);

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace farm_ng
#endif
