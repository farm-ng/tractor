#ifndef FARM_NG_CALIBRATION_APRILTAG_RIG_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_APRILTAG_RIG_CALIBRATOR_H_
#include <array>
#include <string>
#include <unordered_set>
#include <vector>

#include <sophus/se3.hpp>

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/image.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::ApriltagRig;
using farm_ng_proto::tractor::v1::ApriltagRigTagStats;
using farm_ng_proto::tractor::v1::CalibratorCommand;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::SolverStatus;

using Sophus::SE3d;

class EventBus;

struct ApriltagRigModel {
  int root_id;
  std::string rig_name;
  std::string camera_frame_name;
  std::vector<ApriltagDetections> all_detections;
  std::unordered_map<int, SE3d> tag_pose_root;
  std::unordered_map<int, double> tag_size;
  std::unordered_map<int, std::array<Eigen::Vector3d, 4>> points_tag;
  std::vector<Sophus::optional<SE3d>> camera_poses_root;

  std::vector<Image> reprojection_images;

  SolverStatus status;
  std::unordered_map<int, ApriltagRigTagStats> per_tag_stats;
  double rmse;

  void ToMonocularApriltagRigModel(MonocularApriltagRigModel* rig) const;
};

void ModelError(ApriltagRigModel& model);

Sophus::optional<SE3d> EstimateCameraPoseRig(
    const ApriltagRig& rig, const ApriltagDetections& detections);

class ApriltagRigCalibrator {
 public:
  ApriltagRigCalibrator(EventBus* bus,
                        const CalibratorCommand::ApriltagRigStart& rig_start);

  void PoseInit(const std::vector<std::unordered_map<int, SE3d>>& frames,
                std::unordered_map<int, SE3d>& tag_mean_pose_root,
                std::vector<Sophus::optional<SE3d>>& camera_poses_root);

  ApriltagRigModel PoseInitialization();
  void AddFrame(ApriltagDetections detections);

  int NumFrames() const;

  EventBus* bus_;
  std::string rig_name_ = "rig";  // TODO(ethanrublee) set from command/config.
  int root_id_;
  std::unordered_set<int> ids_;
  std::vector<ApriltagDetections> all_detections_;
};  // namespace farm_ng

bool Solve(ApriltagRigModel& model);

}  // namespace farm_ng
#endif
