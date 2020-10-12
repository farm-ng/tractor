#ifndef FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_

#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/resource.pb.h"
#include "farm_ng_proto/tractor/v1/tractor.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::BaseToCameraInitialization;
using farm_ng_proto::tractor::v1::BaseToCameraModel;
using farm_ng_proto::tractor::v1::TractorState;

using farm_ng_proto::tractor::v1::Resource;

void CopyTractorStateToWheelState(
    const TractorState& tractor_state,
    BaseToCameraModel::WheelMeasurement* wheel_measurement);

struct BasePoseCameraSolverOptions {
  bool hold_base_pose_camera_constant = false;
  bool hold_base_parameters_constant = false;
};
BaseToCameraModel SolveBasePoseCamera(
    BaseToCameraModel model,
    BasePoseCameraSolverOptions options = BasePoseCameraSolverOptions());

BaseToCameraModel InitialBaseToCameraModelFromEventLog(
    const BaseToCameraInitialization& initialization,
    const Resource& event_log_resource, const Resource& apriltag_rig_resource);

}  // namespace farm_ng
#endif
