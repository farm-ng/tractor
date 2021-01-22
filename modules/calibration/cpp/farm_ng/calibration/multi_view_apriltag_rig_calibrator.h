#ifndef FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_

#include "farm_ng/calibration/calibrate_multi_view_apriltag_rig.pb.h"
#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/perception/pose_graph.h"

namespace farm_ng {
namespace calibration {

perception::PoseGraph TagRigFromMultiViewDetections(
    std::string tag_rig_name, int root_tag_id,
    MultiViewApriltagRigModel* model);

void CameraRigFromMultiViewDetections(std::string camera_rig_name,
                                      std::string root_camera_name,
                                      const perception::PoseGraph& tag_rig,
                                      MultiViewApriltagRigModel* model);

void ModelError(MultiViewApriltagRigModel* model);
MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    MultiViewApriltagRigModel model);

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config);

}  // namespace calibration
}  // namespace farm_ng
#endif
