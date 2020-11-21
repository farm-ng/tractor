#include "farm_ng/calibration/local_parameterization.h"

#include <ceres/ceres.h>
#include <sophus/se3.hpp>

using Sophus::SE3;
using Sophus::SE3d;
using Sophus::SO3;
using Sophus::SO3d;
using Sophus::Vector6d;

namespace farm_ng {
namespace calibration {

LocalParameterizationAbs::LocalParameterizationAbs(int size) : size_(size) {}

LocalParameterizationAbs::~LocalParameterizationAbs() {}

bool LocalParameterizationAbs::Plus(const double* x, const double* delta,
                                    double* x_plus_delta) const {
  ceres::VectorRef(x_plus_delta, size_) =
      (ceres::ConstVectorRef(x, size_) + ceres::ConstVectorRef(delta, size_))
          .cwiseAbs();

  return true;
}

bool LocalParameterizationAbs::ComputeJacobian(const double* x,
                                               double* jacobian) const {
  ceres::MatrixRef(jacobian, size_, size_).setIdentity();
  return true;
}

int LocalParameterizationAbs::GlobalSize() const { return size_; }

int LocalParameterizationAbs::LocalSize() const { return size_; }

LocalParameterizationSE3::LocalParameterizationSE3() {}
LocalParameterizationSE3::~LocalParameterizationSE3() {}

namespace {

template <typename Scalar>
bool SE3Plus(const Scalar* T_raw, const Scalar* delta_raw,
             Scalar* T_plus_delta_raw) {
  Eigen::Map<typename SE3<Scalar>::Tangent const> delta(delta_raw);
  Eigen::Map<SE3<Scalar> const> const T(T_raw);
  Eigen::Map<SE3<Scalar>> T_plus_delta(T_plus_delta_raw);
  T_plus_delta = T * SE3<Scalar>::exp(delta);
  return true;
}

template <typename Scalar>
bool SO3Plus(const Scalar* T_raw, const Scalar* delta_raw,
             Scalar* T_plus_delta_raw) {
  Eigen::Map<typename SO3<Scalar>::Tangent const> delta(delta_raw);
  Eigen::Map<SO3<Scalar> const> const T(T_raw);
  Eigen::Map<SO3<Scalar>> T_plus_delta(T_plus_delta_raw);
  T_plus_delta = T * SO3<Scalar>::exp(delta);
  return true;
}

// Utility class for computing Dx_this_mul_exp_x_at_0 using
// ceres::internal::AutoDifferentiate
struct SE3PlusAt0 {
  SE3PlusAt0(SE3<double> T) : T_(T) {}
  template <typename Scalar>
  bool operator()(const Scalar* delta_raw, Scalar* T_plus_delta_raw) const {
    SE3<Scalar> T = T_.cast<Scalar>();
    return SE3Plus(T.data(), delta_raw, T_plus_delta_raw);
  }
  SE3<double> T_;
};
}  // namespace

bool LocalParameterizationSE3::Plus(double const* T_raw,
                                    double const* delta_raw,
                                    double* T_plus_delta_raw) const {
  return SE3Plus(T_raw, delta_raw, T_plus_delta_raw);
}

// Jacobian of SE3 plus operation for Ceres
//
// Dx T * exp(x)  with  x=0
//
bool LocalParameterizationSE3::ComputeJacobian(double const* T_raw,
                                               double* jacobian_raw) const {
  // Compared using ceres autodiff versus the generated code in Sophus.
  // This is as fast or faster, and seems to be slight bit more accurate (but
  // really down in the precision bit flipping). This allows for derivative of
  // exp(x)*T or T*exp(x)
  using ParameterDims = typename ceres::SizedCostFunction<7, 6>::ParameterDims;
  std::array<double, 6> zero({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::array<const double*, 1> parameters({zero.data()});
  constexpr int kOutput = 7;
  double output[kOutput];

  ceres::internal::AutoDifferentiate<kOutput, ParameterDims>(
      SE3PlusAt0(Eigen::Map<SE3d const>(T_raw)), parameters.data(), kOutput,
      output, &jacobian_raw);
  return true;
}

int LocalParameterizationSE3::GlobalSize() const {
  return SE3d::num_parameters;
}
int LocalParameterizationSE3::LocalSize() const { return SE3d::DoF; }

LocalParameterizationSO3::LocalParameterizationSO3() {}
LocalParameterizationSO3::~LocalParameterizationSO3() {}

namespace {

// Utility class for computing Dx_this_mul_exp_x_at_0 using
// ceres::internal::AutoDifferentiate
struct SO3PlusAt0 {
  SO3PlusAt0(SO3d T) : T_(T) {}
  template <typename Scalar>
  bool operator()(const Scalar* delta_raw, Scalar* T_plus_delta_raw) const {
    Sophus::SO3<Scalar> T = T_.cast<Scalar>();
    return SO3Plus(T.data(), delta_raw, T_plus_delta_raw);
  }
  SO3d T_;
};
}  // namespace

bool LocalParameterizationSO3::Plus(double const* T_raw,
                                    double const* delta_raw,
                                    double* T_plus_delta_raw) const {
  return SO3Plus(T_raw, delta_raw, T_plus_delta_raw);
}

// Jacobian of SE3 plus operation for Ceres
//
// Dx T * exp(x)  with  x=0
//
bool LocalParameterizationSO3::ComputeJacobian(double const* T_raw,
                                               double* jacobian_raw) const {
  // Compared using ceres autodiff versus the generated code in Sophus.
  // This is as fast or faster, and seems to be slight bit more accurate (but
  // really down in the precision bit flipping). This allows for derivative of
  // exp(x)*T or T*exp(x)
  using ParameterDims = typename ceres::SizedCostFunction<4, 3>::ParameterDims;
  std::array<double, 3> zero({0.0, 0.0, 0.0});
  std::array<const double*, 1> parameters({zero.data()});
  constexpr int kOutput = 4;
  double output[kOutput];

  ceres::internal::AutoDifferentiate<kOutput, ParameterDims>(
      SO3PlusAt0(Eigen::Map<SO3d const>(T_raw)), parameters.data(), kOutput,
      output, &jacobian_raw);
  return true;
}

int LocalParameterizationSO3::GlobalSize() const {
  return SO3d::num_parameters;
}
int LocalParameterizationSO3::LocalSize() const {
  return SO3d::DoF;
}  // return local_size_; }

void AddSE3ParameterBlockSubsetTranslation(
    ceres::Problem* problem, double* parameter_block,
    const std::vector<int>& translation_subset) {
  problem->AddParameterBlock(
      parameter_block, SE3d::num_parameters,
      new ceres::ProductParameterization(
          new LocalParameterizationSO3(),
          new ceres::SubsetParameterization(3, translation_subset)));
}

}  // namespace calibration
}  // namespace farm_ng
