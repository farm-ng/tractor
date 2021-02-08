#ifndef FARM_NG_PERCEPTION_INTEL_RS2_UTILS_H_
#define FARM_NG_PERCEPTION_INTEL_RS2_UTILS_H_
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>

#include "farm_ng/perception/camera_model.pb.h"

namespace farm_ng {
namespace perception {

cv::Mat RS2FrameToMat(const rs2::frame& f);

void SetCameraModelFromRs(CameraModel* out, const rs2_intrinsics& intrinsics);
}  // namespace perception
}  // namespace farm_ng
#endif
