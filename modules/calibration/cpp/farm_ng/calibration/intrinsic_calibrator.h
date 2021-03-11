#ifndef FARM_NG_CALIBRATION_INTRINSIC_CALIBRATOR_H_
#define FARM_NG_CALIBRATION_INTRINSIC_CALIBRATOR_H_
#include "farm_ng/calibration/calibrate_intrinsics.pb.h"
#include "farm_ng/calibration/intrinsic_model.pb.h"
#include "farm_ng/core/ipc.h"

namespace farm_ng {
namespace calibration {


void ModelError(IntrinsicModel* model, bool disable_reprojection_images);

IntrinsicModel SolveIntrinsicsModel(IntrinsicModel model);

IntrinsicModel InitialIntrinsicModelFromConfig(
    const CalibrateIntrinsicsConfiguration& config);

}  // namespace calibration
}  // namespace farm_ng
#endif
