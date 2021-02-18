#ifndef FARM_NG_PERCEPTION_TENSOR_H_
#define FARM_NG_PERCEPTION_TENSOR_H_
#include <Eigen/Core>
#include <string>

#include "farm_ng/perception/tensor.pb.h"

namespace farm_ng::perception {

void EigenToTensor(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::ColMajor>& x,
                   std::string rows_name, std::string cols_name, Tensor* out);

Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
TensorToEigenMapXd(const farm_ng::perception::Tensor& x);

void TensorToEigen(const farm_ng::perception::Tensor& x,
                   Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::ColMajor>* out);
}  // namespace farm_ng::perception

#endif
