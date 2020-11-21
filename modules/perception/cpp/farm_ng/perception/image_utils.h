#ifndef FARM_NG_IMAGE_UTILS_H_
#define FARM_NG_IMAGE_UTILS_H_
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace farm_ng {
namespace perception {

inline cv::Mat ConstructGridImage(const std::vector<cv::Mat>& images,
                                  cv::Size out_size, int n_cols) {
  cv::Mat grid_image = cv::Mat::zeros(out_size, CV_8UC3);

  int n_rows = std::ceil(images.size() / float(n_cols));
  size_t i = 0;
  int target_width = grid_image.size().width / n_cols;
  int target_height = grid_image.size().height / n_rows;
  for (int row = 0; row < n_rows && i < images.size(); ++row) {
    for (int col = 0; col < n_cols && i < images.size(); ++col, ++i) {
      cv::Mat image_i = images[i];
      float width_ratio = target_width / float(image_i.size().width);
      float height_ratio = target_height / float(image_i.size().height);
      float resize_ratio = std::min(width_ratio, height_ratio);
      int i_width = image_i.size().width * resize_ratio;
      int i_height = image_i.size().height * resize_ratio;
      cv::Rect roi(col * target_width, row * target_height, i_width, i_height);
      cv::Mat color_i;
      if (image_i.channels() == 1) {
        cv::cvtColor(image_i, color_i, cv::COLOR_GRAY2BGR);
      } else {
        color_i = image_i;
        CHECK_EQ(color_i.channels(), 3);
      }

      cv::resize(color_i, grid_image(roi), roi.size());
    }
  }
  return grid_image;
}

}  // namespace perception
}  // namespace farm_ng
#endif
