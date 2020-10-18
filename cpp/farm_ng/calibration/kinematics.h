#ifndef FARM_NG_CALIBRATION_KINEMATICS_H_
#define FARM_NG_CALIBRATION_KINEMATICS_H_
#include <sophus/se3.hpp>

namespace farm_ng {

template <typename T>
Sophus::SE3<T> TractorPoseDelta(const T& wheel_radius, const T& wheel_baseline,
                                const T& vel_left, const T& vel_right,
                                const T& dt) {
  T v = (wheel_radius / T(2.0)) * (vel_left + vel_right);
  T w = (wheel_radius / wheel_baseline) * (vel_right - vel_left);
  Eigen::Matrix<T, 6, 1> x;
  x << v * dt, T(0), T(0), T(0), T(0), w * dt;
  return Sophus::SE3<T>::exp(x);
}

template <typename T>
Sophus::SE3<T> TractorPoseDelta(
    const T& wheel_radius, const T& wheel_baseline,
    const BaseToCameraModel::WheelMeasurement& wheel_measurement) {
  T vel_left(wheel_measurement.wheel_velocity_rads_left());
  T vel_right(wheel_measurement.wheel_velocity_rads_right());
  T dt(wheel_measurement.dt());
  return TractorPoseDelta(wheel_radius, wheel_baseline, vel_left, vel_right,
                          dt);
}

template <typename T, typename IteratorT>
Sophus::SE3<T> TractorStartPoseTractorEnd(const T& wheel_radius,
                                          const T& wheel_baseline,
                                          IteratorT begin_wheel_measurement,
                                          IteratorT end_wheel_measurement) {
  Sophus::SE3<T> tractor_start_pose_tractor_end =
      Sophus::SE3d::rotZ(0).cast<T>();
  while (begin_wheel_measurement != end_wheel_measurement) {
    const auto& wheel_measurement = *(begin_wheel_measurement++);
    tractor_start_pose_tractor_end =
        tractor_start_pose_tractor_end *
        TractorPoseDelta<T>(wheel_radius, wheel_baseline, wheel_measurement);
  }
  return tractor_start_pose_tractor_end;
}

template <typename T>
Sophus::SE3<T> TractorStartPoseTractorEnd(
    const T& wheel_radius, const T& wheel_baseline,
    const BaseToCameraModel::Sample& sample) {
  return TractorStartPoseTractorEnd(wheel_radius, wheel_baseline,
                                    sample.wheel_measurements().begin(),
                                    sample.wheel_measurements().end());
}

}  // namespace farm_ng
#endif
