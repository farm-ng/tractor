#ifndef FARM_NG_CALIBRATION_APRILTAG_H_
#define FARM_NG_CALIBRATION_APRILTAG_H_
#include <array>

#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/camera_model.pb.h"

namespace farm_ng {

namespace core {
class EventBus;
}

namespace perception {

std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection);

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection);

double TagSize(const TagLibrary& tag_library, int tag_id);

// Adds the tag id and sizes contained in ApriltagRig to the ApriltagConfig.
void AddApriltagRigToApriltagConfig(const ApriltagRig& rig,
                                    ApriltagConfig* config);

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
  bool AddApriltags(const ApriltagDetections& detections, int steady_count = 5,
                    int window_size = 7);

 private:
  cv::Mat mask_;
  bool once_;
};

class ApriltagDetector {
 public:
  ApriltagDetector(const CameraModel& camera_model,
                   farm_ng::core::EventBus* event_bus = nullptr,
                   const ApriltagConfig* config = nullptr);

  ~ApriltagDetector();

  void Close();

  ApriltagDetections Detect(const cv::Mat& gray,
                            const google::protobuf::Timestamp& stamp);

  // Detects apriltags, and looks up the depth value at each point.
  // Precondition: the depthmap is the same size as gray
  // Precondition: the depthmap is of type CV_32FC1.
  ApriltagDetections Detect(const cv::Mat& gray, const cv::Mat& depthmap,
                            const google::protobuf::Timestamp& stamp);

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

// Helper class for looking up rig frame names based on tag ids.  Useful in
// combination with the PoseGraph which uses string names for looking up poses.
class ApriltagRigIdMap {
 public:
  ApriltagRigIdMap() = default;

  struct FrameNames {
    std::string tag_frame;
    std::string rig_frame;
  };

  // Adds all the nodes in the given ApriltagRig to the map.
  // Precondition: all tag ids must be unique, within the given rig and w.r.t.
  // any previously added rig.
  void AddRig(const ApriltagRig& rig) {
    for (const ApriltagRig::Node& node : rig.nodes()) {
      insert(rig, node);
    }
  }

  std::optional<FrameNames> GetFrameNames(int tag_id) {
    auto it_name = id_frame_map_.find(tag_id);
    if (it_name != id_frame_map_.end()) {
      return it_name->second;
    }
    return std::nullopt;
  }

 private:
  void insert(const ApriltagRig& rig, const ApriltagRig::Node& node) {
    auto it_inserted = id_frame_map_.insert(
        std::make_pair(node.id(), FrameNames{node.frame_name(), rig.name()}));
    // Here we prevent any duplicate tag ids.
    CHECK(it_inserted.second)
        << "Failed to insert duplicate node: " << node.ShortDebugString();
  }
  std::map<int, FrameNames> id_frame_map_;
};

}  // namespace perception
}  // namespace farm_ng
#endif
