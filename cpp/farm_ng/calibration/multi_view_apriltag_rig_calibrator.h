#ifndef FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_

#include "farm_ng_proto/tractor/v1/calibrate_multi_view_apriltag_rig.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::CalibrateMultiViewApriltagRigConfiguration;
using farm_ng_proto::tractor::v1::MultiViewApriltagRigModel;

MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    MultiViewApriltagRigModel model);

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config);

}  // namespace farm_ng
#endif
