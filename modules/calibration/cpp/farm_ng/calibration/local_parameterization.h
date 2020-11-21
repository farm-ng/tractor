#ifndef FARM_NG_CALIBRATION_LOCAL_PARAMETERIZATION_SE3_H_
#define FARM_NG_CALIBRATION_LOCAL_PARAMETERIZATION_SE3_H_

#include <array>

#include <ceres/local_parameterization.h>
#include <ceres/problem.h>

namespace farm_ng {
namespace calibration {

class LocalParameterizationAbs : public ceres::LocalParameterization {
 public:
  LocalParameterizationAbs(int size);

  virtual ~LocalParameterizationAbs();

  bool Plus(const double* x, const double* delta, double* x_plus_delta) const;

  bool ComputeJacobian(const double* x, double* jacobian) const;

  virtual int GlobalSize() const;

  virtual int LocalSize() const;
  int size_;
};

// https://github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
class LocalParameterizationSO3 : public ceres::LocalParameterization {
 public:
  explicit LocalParameterizationSO3();
  ~LocalParameterizationSO3();

  // SO3 plus operation for Ceres
  //
  //  a_T_b' = a_T_b * exp(b_x_b')
  //
  bool Plus(double const* T_raw, double const* delta_raw,
            double* T_plus_delta_raw) const override;

  // Jacobian of SO3 plus operation for Ceres
  //
  // Dx a_T_b * exp(b_x_b')  with  b_x_b'=0
  //
  bool ComputeJacobian(double const* T_raw,
                       double* jacobian_raw) const override;

  int GlobalSize() const override;
  int LocalSize() const override;

 private:
};

// github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  explicit LocalParameterizationSE3();
  ~LocalParameterizationSE3();

  // SE3 plus operation for Ceres
  //
  //  a_T_b' = a_T_b * exp(b_x_b')
  //
  bool Plus(double const* T_raw, double const* delta_raw,
            double* T_plus_delta_raw) const override;

  // Jacobian of SO3 plus operation for Ceres
  //
  // Dx a_T_b * exp(b_x_b')  with  b_x_b'=0
  //
  bool ComputeJacobian(double const* T_raw,
                       double* jacobian_raw) const override;

  int GlobalSize() const override;
  int LocalSize() const override;

 private:
};

// Add an SE3 parameter block {qx, qy, qz, qw, tx, ty, tz} and enable holding a
// subset of the translation constant.
void AddSE3ParameterBlockSubsetTranslation(
    ceres::Problem* problem, double* parameter_block,
    const std::vector<int>& translation_subset);

}  // namespace calibration
}  // namespace farm_ng

#endif  // FARM_NG_CALIBRATION_LOCAL_PARAMETERIZATION_SE3_H_
