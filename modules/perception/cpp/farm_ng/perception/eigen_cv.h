#ifndef FARM_NG_CALIBRATION_CALIBRATION_EIGEN_CV_H_
#define FARM_NG_CALIBRATION_CALIBRATION_EIGEN_CV_H_
#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace farm_ng {
namespace perception {

template <typename Scalar>
inline cv::Point2f EigenToCvPoint2f(const Eigen::Matrix<Scalar, 2, 1>& x) {
  return cv::Point2f(x.x(), x.y());
}

template <typename Scalar>
inline cv::Point EigenToCvPoint(const Eigen::Matrix<Scalar, 2, 1>& x) {
  return cv::Point(int(x.x() + 0.5), int(x.y() + 0.5));
}

}  // namespace perception
}  // namespace farm_ng
#endif
