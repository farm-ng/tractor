#include "farm_ng/perception/camera_model.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

using namespace farm_ng;

TEST(camera_model, smoke) {
  CameraModel model = DefaultFishEyeT265CameraModel();

  Eigen::Vector2d x =
      ProjectPointToPixel(model, Eigen::Vector3d(0.0, 0.0, 2.0));
  LOG(INFO) << x.transpose();
  EXPECT_FLOAT_EQ(model.cx(), x[0]);
  EXPECT_FLOAT_EQ(model.cy(), x[1]);
}

TEST(apriltag_size_study, smoke) {
  CameraModel model = DefaultFishEyeT265CameraModel();

  int n_widths = 10;
  double min_width = 0.16;
  double max_width = 1.0;

  int n_depths = 10;

  double min_depth = 1.0;
  double max_depth = 10.0;

  for (int j = 0; j < n_widths; ++j) {
    double width =
        (j / double(n_widths - 1)) * (max_width - min_width) + min_width;

    for (int i = 0; i < n_depths; ++i) {
      double depth =
          (i / double(n_depths - 1)) * (max_depth - min_depth) + min_depth;

      Eigen::Vector2d x1 =
          ProjectPointToPixel(model, Eigen::Vector3d(-width / 2.0, 0.0, depth));
      Eigen::Vector2d x2 =
          ProjectPointToPixel(model, Eigen::Vector3d(width / 2.0, 0.0, depth));

      std::cout << "width: " << width << " depth: " << depth
                << " pixels: " << (x1 - x2).norm() << std::endl;
    }
  }
}
