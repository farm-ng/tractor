#ifndef FARM_NG_CALIBRATION_APRILTAG_RIG_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_APRILTAG_RIG_CALIBRATOR_H_
#include <array>
#include <string>
#include <unordered_set>
#include <vector>

#include <sophus/se3.hpp>

#include "farm_ng/calibration/calibrate_apriltag_rig.pb.h"
#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/image.pb.h"

namespace farm_ng {
namespace calibration {

struct ApriltagRigModel {
  int root_id;
  std::string rig_name;
  std::string camera_frame_name;
  std::vector<farm_ng::perception::ApriltagDetections> all_detections;
  std::unordered_map<int, Sophus::SE3d> tag_pose_root;
  std::unordered_map<int, double> tag_size;
  std::unordered_map<int, std::array<Eigen::Vector3d, 4>> points_tag;
  std::vector<Sophus::optional<Sophus::SE3d>> camera_poses_root;

  std::vector<farm_ng::perception::Image> reprojection_images;

  SolverStatus status;
  std::unordered_map<int, ApriltagRigTagStats> per_tag_stats;
  double rmse;

  void ToMonocularApriltagRigModel(MonocularApriltagRigModel* rig) const;
};

void ModelError(ApriltagRigModel& model);

Sophus::optional<farm_ng::perception::NamedSE3Pose> EstimateCameraPoseRig(
    const farm_ng::perception::ApriltagRig& rig,
    const farm_ng::perception::ApriltagDetections& detections);

class ApriltagRigCalibrator {
 public:
  ApriltagRigCalibrator(const CalibrateApriltagRigConfiguration& config);

  void PoseInit(
      const std::vector<std::unordered_map<int, Sophus::SE3d>>& frames,
      std::unordered_map<int, Sophus::SE3d>& tag_mean_pose_root,
      std::vector<Sophus::optional<Sophus::SE3d>>& camera_poses_root);

  ApriltagRigModel PoseInitialization();
  void AddFrame(farm_ng::perception::ApriltagDetections detections);

  int NumFrames() const;

  std::string rig_name_ = "rig";  // TODO(ethanrublee) set from command/config.
  int root_id_;
  std::unordered_set<int> ids_;
  std::vector<farm_ng::perception::ApriltagDetections> all_detections_;
};

bool Solve(ApriltagRigModel* model);

}  // namespace calibration
}  // namespace farm_ng
#endif
