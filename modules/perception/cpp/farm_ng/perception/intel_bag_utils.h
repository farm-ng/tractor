#ifndef FARM_NG_INTEL_BAG_UTILS_H_
#define FARM_NG_INTEL_BAG_UTILS_H_
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>

namespace farm_ng {
namespace perception {

namespace {

// Convert rs2::frame to cv::Mat
// Supported frame formats include BGR, RGB, depth maps, and disparity
// Any RGB frame is converted to BGR format
// https://raw.githubusercontent.com/IntelRealSense/librealsense/master/wrappers/opencv/cv-helpers.hpp
cv::Mat RS2FrameToMat(const rs2::frame& f) {
  using namespace cv;
  using namespace rs2;

  auto vf = f.as<video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();
  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    Mat r_bgr;
    cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
    return r_bgr;
  } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
    return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
    return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
  }

  throw std::runtime_error("Frame format is not supported yet!");
}

void SetCameraModelFromRs(CameraModel* out, const rs2_intrinsics& intrinsics) {
  out->set_image_width(intrinsics.width);
  out->set_image_height(intrinsics.height);
  out->set_cx(intrinsics.ppx);
  out->set_cy(intrinsics.ppy);
  out->set_fx(intrinsics.fx);
  out->set_fy(intrinsics.fy);
  switch (intrinsics.model) {
    case RS2_DISTORTION_KANNALA_BRANDT4:
      out->set_distortion_model(CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4);
      break;
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      out->set_distortion_model(
          CameraModel::DISTORTION_MODEL_INVERSE_BROWN_CONRADY);
      break;
    default:
      CHECK(false) << "Unhandled intrinsics model: "
                   << rs2_distortion_to_string(intrinsics.model);
  }
  for (int i = 0; i < 5; ++i) {
    out->add_distortion_coefficients(intrinsics.coeffs[i]);
  }
}
}  // namespace
}  // namespace perception
}  // namespace farm_ng
#endif
