#ifndef FARM_NG_CALIBRATION_MULTI_VIEW_APRILTAG_RIG_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_MULTI_VIEW_APRILTAG_RIG_CALIBRATOR_H_

#include "farm_ng/calibration/calibrate_multi_view_apriltag_rig.pb.h"
#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/perception/pose_graph.h"

namespace farm_ng::calibration {

perception::PoseGraph TagRigFromMultiViewDetections(
    std::string tag_rig_name, int root_tag_id,
    MultiViewApriltagRigModel* model);

void CameraRigFromMultiViewDetections(std::string camera_rig_name,
                                      std::string root_camera_name,
                                      const perception::PoseGraph& tag_rig,
                                      MultiViewApriltagRigModel* model);

void ModelError(MultiViewApriltagRigModel* model,
                bool output_reprojection_images);
MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    MultiViewApriltagRigModel model);

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config);

std::optional<std::tuple<perception::NamedSE3Pose, double,
                         std::vector<ApriltagRigTagStats>>>
EstimateMultiViewCameraRigPoseApriltagRig(
    const perception::MultiViewCameraRig& camera_rig,
    const perception::ApriltagRig& apriltag_rig,
    const perception::MultiViewApriltagDetections& mv_detections);

}  // namespace farm_ng::calibration
#endif
