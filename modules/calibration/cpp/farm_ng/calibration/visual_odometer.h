#ifndef FARM_NG_CALIBRATION_VISUAL_ODOMETER_H_
#define FARM_NG_CALIBRATION_VISUAL_ODOMETER_H_

#include <Eigen/Core>
#include <unordered_map>

#include <ceres/problem.h>
#include <google/protobuf/timestamp.pb.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <sophus/se3.hpp>

#include "farm_ng/calibration/calibrate_base_to_camera.pb.h"
#include "farm_ng/calibration/flow_book_keeper.h"
#include "farm_ng/perception/camera_model.pb.h"
#include "farm_ng/perception/time_series.h"

namespace farm_ng {
namespace calibration {

struct VisualOdometerResult {
  farm_ng::perception::NamedSE3Pose odometry_vo_pose_base;
  farm_ng::perception::NamedSE3Pose base_pose_goal;
};

class VisualOdometer {
 public:
  VisualOdometer(const farm_ng::perception::CameraModel& camera_model,
                 const BaseToCameraModel& base_to_camera_model,
                 size_t max_history);

  void AddWheelMeasurements(
      const BaseToCameraModel::WheelMeasurement& measurements);

  VisualOdometerResult AddImage(cv::Mat image,
                                google::protobuf::Timestamp stamp);

  void DumpFlowPointsWorld(std::string ply_path);

  void SetGoal();

  void AdjustGoalAngle(double theta);

  cv::Mat GetDebugImage() const { return debug_image_; }

 private:
  void SolvePose(bool debug = true);

  void AddFlowBlockToProblem(ceres::Problem* problem,
                             const FlowBlock& flow_block);

  void AddFlowImageToProblem(FlowImage* flow_image, ceres::Problem* problem,
                             FlowBlocks* flow_blocks);
  farm_ng::perception::CameraModel camera_model_;
  FlowBookKeeper flow_;
  BaseToCameraModel base_to_camera_model_;

  Sophus::SE3d base_pose_camera_;
  Sophus::SE3d odometry_pose_base_;

  std::optional<uint64_t> goal_image_id_;
  Sophus::SE3d camera_pose_base_goal_;
  farm_ng::perception::TimeSeries<BaseToCameraModel::WheelMeasurement>
      wheel_measurements_;

  cv::Mat debug_image_;
};

}  // namespace calibration
}  // namespace farm_ng
#endif
