#ifndef FARM_NG_CALIBRATION_LOCAL_PARAMETERIZATION_SE3_H_
#define FARM_NG_CALIBRATION_LOCAL_PARAMETERIZATION_SE3_H_

#include <ceres/local_parameterization.h>
#include <sophus/se3.hpp>

namespace farm_ng {

using Sophus::SE3d;
using Sophus::Vector6d;

// https://github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE3() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<SE3d const> const T(T_raw);
    Eigen::Map<Vector6d const> const delta(delta_raw);
    Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * SE3d::exp(delta);
    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // Dx T * exp(x)  with  x=0
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
    Eigen::Map<SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
        jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual int GlobalSize() const { return SE3d::num_parameters; }

  virtual int LocalSize() const { return SE3d::DoF; }
};

}  // namespace farm_ng

#endif  // FARM_NG_CALIBRATION_LOCAL_PARAMETERIZATION_SE3_H_
