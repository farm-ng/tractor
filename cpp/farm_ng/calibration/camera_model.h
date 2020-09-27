#ifndef FARM_NG_CALIBRATION_CAMERA_MODEL_H_
#define FARM_NG_CALIBRATION_CAMERA_MODEL_H_

#include <numeric>

#include <glog/logging.h>
#include <Eigen/Dense>

#include "farm_ng_proto/tractor/v1/camera_model.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::CameraModel;

// Given a point in 3D space, compute the corresponding pixel coordinates in an
//  image with no distortion or forward distortion coefficients produced by the
//  same camera
//
//  This is compatable with autodiff using ceres jet types, except that it will
//  not support solving for the camera model itself.
template <class T>
Eigen::Matrix<T, 2, 1> ProjectPointToPixel(
    const CameraModel& camera, const Eigen::Matrix<T, 3, 1>& point) {
  using std::atan;
  using std::sqrt;
  const T eps(std::numeric_limits<float>::epsilon());
  T x = point.x() / point.z();

  T y = point.y() / point.z();

  CHECK_EQ(camera.distortion_model(),
           CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4);
  if (camera.distortion_model() ==
      CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4) {
    // Model copied from librealsense:
    // https://github.com/IntelRealSense/librealsense/blob/0adceb9dc6fce63c348346e1aef1b63c052a1db9/include/librealsense2/rsutil.h#L63
    T r = sqrt(x * x + y * y);
    if (r < eps) {
      r = eps;
    }
    T theta = atan(r);
    T theta2 = theta * theta;
    T series =
        T(1) +
        theta2 *
            (T(camera.distortion_coefficients(0)) +
             theta2 *
                 (T(camera.distortion_coefficients(1)) +
                  theta2 * (T(camera.distortion_coefficients(2)) +
                            theta2 * T(camera.distortion_coefficients(3)))));
    T rd = theta * series;
    x *= rd / r;
    y *= rd / r;
  }

  return Eigen::Matrix<T, 2, 1>(x * T(camera.fx()) + T(camera.cx()),
                                y * T(camera.fy()) + T(camera.cy()));
}

CameraModel DefaultFishEyeT265CameraModel();

}  // namespace farm_ng

#endif  // FARM_NG_CALIBRATION_CAMERA_MODEL_H_
