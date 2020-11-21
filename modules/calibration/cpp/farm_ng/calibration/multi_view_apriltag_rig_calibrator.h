#ifndef FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_BASE_TO_CAMERA_CALIBRATOR_H_

#include "farm_ng/calibration/calibrate_multi_view_apriltag_rig.pb.h"
#include "farm_ng/calibration/calibrator.pb.h"

namespace farm_ng {
namespace calibration {

MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    MultiViewApriltagRigModel model);

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config);

}  // namespace calibration
}  // namespace farm_ng
#endif
