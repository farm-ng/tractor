#ifndef FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_

#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/core/resource.pb.h"
#include "farm_ng/tractor/tractor.pb.h"

namespace farm_ng {
namespace tractor {

void CopyTractorStateToWheelState(
    const TractorState& tractor_state,
    farm_ng::calibration::BaseToCameraModel::WheelMeasurement*
        wheel_measurement);

struct BasePoseCameraSolverOptions {
  bool hold_base_pose_camera_constant = false;
  bool hold_base_parameters_constant = false;
};
farm_ng::calibration::BaseToCameraModel SolveBasePoseCamera(
    farm_ng::calibration::BaseToCameraModel model,
    BasePoseCameraSolverOptions options = BasePoseCameraSolverOptions());

farm_ng::calibration::BaseToCameraModel InitialBaseToCameraModelFromEventLog(
    const farm_ng::calibration::BaseToCameraInitialization& initialization,
    const farm_ng::core::Resource& event_log_resource,
    const farm_ng::core::Resource& apriltag_rig_resource);

}  // namespace tractor
}  // namespace farm_ng
#endif
