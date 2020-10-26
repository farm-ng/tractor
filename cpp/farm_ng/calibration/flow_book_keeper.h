#ifndef FARM_NG_CALIBRATION_FLOW_BOOK_KEEPER_H_
#define FARM_NG_CALIBRATION_FLOW_BOOK_KEEPER_H_

#include <Eigen/Core>
#include <unordered_map>

#include <google/protobuf/timestamp.pb.h>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "farm_ng/calibration/time_series.h"

#include "farm_ng_proto/tractor/v1/camera_model.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::CameraModel;

// An image space measurement of a flow point.
struct FlowPointImage {
  // This id is the global id of the 3d point which projects to produce this
  // measurement, and matches the id in FlowPointWorld.
  uint64_t id;
  // Image space measurement in pixels.
  Eigen::Vector2f point_image;
};

// 3D estimate of point observed across multiple views.
struct FlowPointWorld {
  // Globally unique id. FlowPointImage with the same id correspond to this
  // FlowPointWorld.
  uint64_t id;
  // The point in some world reference frame.  The world reference frame is
  // defined by the solver/tracker.
  Eigen::Vector3d point_world;
  // FlowImage ids where this FlowPointWorld is observed. ``std::set`` because
  // we want it ordered by ID which also happens to be chronologic.
  std::set<uint64_t> image_ids;
  // The computed RMSE of the FlowPointWorld across all observations.  This is
  // populated by the solver/tracker - if 0 assume it hasn't been set.
  double rmse = 0;
};

struct FlowImage {
  uint64_t id;
  google::protobuf::Timestamp stamp;
  Sophus::SE3d camera_pose_world;
  std::optional<cv::Mat> image;
  std::optional<std::vector<cv::Mat>> pyramid;
  cv::Mat debug_trails;
  std::unordered_map<uint64_t, FlowPointImage> flow_points;
};

struct FlowBlock {
  FlowImage* flow_image;
  FlowPointWorld* flow_point_world;
  FlowPointImage flow_point_image;
};

typedef std::unordered_map<uint64_t, std::vector<FlowBlock>> FlowBlocks;

class FlowBookKeeper {
 public:
  FlowBookKeeper(CameraModel camera_model, size_t max_history);

  // Add an image, assumed to be from the same camera, and close to periodic,
  // meaning from the same consecutive time series of images captured from the
  // camera.
  uint64_t AddImage(cv::Mat image, google::protobuf::Timestamp stamp,
                    Sophus::SE3d world_pose_camera, bool debug);

  void RemoveBlock(const FlowBlock& block);
  void RemovePointWorld(uint64_t id);
  void RemoveFlowImage(uint64_t id);

  uint64_t LastImageId() const;

  const FlowImage* EarliestFlowImage() const;
  const FlowImage* PreviousFlowImage() const;
  FlowImage* MutablePreviousFlowImage();

  FlowImage* MutableFlowImage(uint64_t id);

  FlowPointWorld* MutableFlowPointWorld(uint64_t id);

  const std::unordered_map<uint64_t, FlowPointWorld>& FlowPointsWorld() const;

  cv::Scalar Color(uint64_t id) const;

  cv::Mat GetDebugImage() const { return debug_image_; }

 private:
  void FlowFromPrevious(FlowImage* flow_image, bool debug = false);

  std::pair<std::vector<cv::Point2f>, std::vector<uint64_t>> GetFlowCvPoints(
      const FlowImage& flow_image) const;
  void RenderMaskOfFlowPoints(const FlowImage& flow_image, cv::Mat* mask,
                              int window_size);
  FlowPointImage GenFlowPoint(FlowImage* flow_image, cv::Point2f x);

  void DetectGoodCorners(FlowImage* flow_image);
  // Maximum number of images to keep in memory
  size_t max_history_;

  CameraModel camera_model_;
  cv::Mat lens_exclusion_mask_;
  std::vector<cv::Scalar> colors_;
  uint64_t image_id_gen_ = 0;
  uint64_t flow_id_gen_ = 0;
  std::unordered_map<uint64_t, FlowPointWorld> flow_points_world_;
  std::unordered_map<uint64_t, FlowImage> flow_images_;
  std::set<uint64_t> flow_image_ids_;
  cv::Mat debug_image_;
  cv::Size flow_window_;
  int flow_max_levels_;
};
}  // namespace farm_ng
#endif
