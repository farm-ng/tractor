
#include <farm_ng/perception/point_cloud.h>

#include <glog/logging.h>
#include <fstream>

#include <farm_ng/perception/tensor.h>

namespace farm_ng::perception {
// Finds point indices within +- x, +- y, +- z
std::tuple<std::vector<int>, Eigen::Vector3d> AxisAlignedBoxFilter(
    const Eigen::Matrix3Xd& points, double x, double y, double z) {
  std::vector<int> indices;
  Eigen::Vector3d m(0, 0, 0);
  for (int i = 0; i < points.cols(); ++i) {
    auto p_i = points.col(i);
    if (p_i.x() > -x && p_i.x() < x &&  // x
        p_i.y() > -y && p_i.y() < y &&  // y
        p_i.z() > -z && p_i.z() < z) {
      m += p_i.cwiseProduct(p_i);
      indices.push_back(i);
    }
  }
  if (indices.size() > 0) {
    m = (m / indices.size()).array().sqrt();
  }
  return {indices, m};
}

Eigen::Matrix3Xd SelectPoints(const Eigen::MatrixXd& points,
                              const std::vector<int>& indices) {
  Eigen::Matrix3Xd out(3, indices.size());

  for (int i = 0; i < out.cols(); ++i) {
    CHECK_LT(indices[i], points.cols());
    CHECK_GE(indices[i], 0);
    out.col(i) = points.col(indices[i]);
  }
  return out;
}

Eigen::Map<const Eigen::MatrixXd> PointCloudGetData(
    const perception::PointCloud& cloud, const std::string& name) {
  for (auto& point_data : cloud.point_data()) {
    CHECK_EQ(point_data.shape_size(), 2);
    if (point_data.shape(0).name() == name) {
      return perception::TensorToEigenMapXd(point_data);
    }
  }
  LOG(FATAL) << "No point_data named: " << name;
  return Eigen::Map<const Eigen::MatrixXd>(nullptr, 0, 0);
}

void SavePly(const std::string& ply_path, const Eigen::MatrixXd& points) {
  std::ofstream out(ply_path);
  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "element vertex " << points.cols() << "\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "end_header\n";
  for (int i = 0; i < points.cols(); ++i) {
    auto p = points.col(i);
    out << float(p.x()) << " " << float(p.y()) << " " << float(p.z()) << "\n";
  }
  out.close();
}
}  // namespace farm_ng::perception
