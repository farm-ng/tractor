#ifndef FARM_NG_CALIBRATION_EPIPOLAR_H_
#define FARM_NG_CALIBRATION_EPIPOLAR_H_
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace farm_ng {
namespace calibration {

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> SkewMatrix(const Eigen::Matrix<Scalar, 3, 1>& x) {
  Eigen::Matrix<Scalar, 3, 3> c;
  c << Scalar(0), -x.z(), x.y(),  //
      x.z(), Scalar(0), -x.x(),   //
      -x.y(), x.x(), Scalar(0);   //
  return c;
}

// https://web.stanford.edu/class/cs231a/course_notes/03-epipolar-geometry.pdf
//
// Given b_pose_a, point_b = b_pose_a*point_a
// Returns Essential matrix (b_E_a) such that:
//
//   point_b.transpose().dot(b_E_a.dot(point_a)) == 0
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> EssentialMatrix(
    const Sophus::SE3<Scalar>& b_pose_a) {
  Eigen::Matrix<Scalar, 3, 3> b_E_a =
      SkewMatrix(b_pose_a.translation()) * b_pose_a.rotationMatrix();
  return b_E_a;
}

template <typename Scalar>
Scalar EpipolarDistance(const Eigen::Matrix<Scalar, 3, 1>& point_a,
                        const Eigen::Matrix<Scalar, 3, 1>& point_b,
                        const Sophus::SE3<Scalar>& a_pose_b) {
  Eigen::Matrix<Scalar, 3, 3> a_E_b = EssentionalMatrix(a_pose_b);
  Eigen::Matrix<Scalar, 3, 1> epiline_a = a_E_b * point_b;
  Scalar norm = epiline_a.template head<2>().norm();
  if (norm > Scalar(1e-5)) {
    epiline_a /= norm;
    return point_a.dot(epiline_a);
  } else {
    return (point_a - point_b).norm();
  }
}

struct EpipolarCostFunctor {
  EpipolarCostFunctor(Eigen::Vector3d point_image_rect_start,
                      Eigen::Vector3d point_image_rect_end)
      : point_image_rect_start_(point_image_rect_start),
        point_image_rect_end_(point_image_rect_end) {}
  template <class T>
  bool operator()(T const* const raw_camera_start_pose_camera_end,
                  T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const camera_start_pose_camera_end(
        raw_camera_start_pose_camera_end);
    raw_residuals[0] = EpipolarDistance<T>(point_image_rect_start_.cast<T>(),
                                           point_image_rect_end_.cast<T>(),
                                           camera_start_pose_camera_end);
    return true;
  }

  Eigen::Vector3d point_image_rect_start_;
  Eigen::Vector3d point_image_rect_end_;
};

}  // namespace calibration
}  // namespace farm_ng
#endif
