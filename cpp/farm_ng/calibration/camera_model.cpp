#include "farm_ng/calibration/camera_model.h"

namespace farm_ng {

CameraModel DefaultFishEyeT265CameraModel() {
  CameraModel model;
  model.set_image_width(848);
  model.set_image_height(800);
  model.set_cx((model.image_width() - 1) / 2.0);
  model.set_cy((model.image_height() - 1) / 2.0);
  model.set_fx(286);  // close to a t265 focal length
  model.set_fy(model.fx());
  model.set_distortion_model(CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4);
  // copied from a t265
  model.add_distortion_coefficients(-0.0080617731437087059);
  model.add_distortion_coefficients(0.04318523034453392);
  model.add_distortion_coefficients(-0.039864420890808105);
  model.add_distortion_coefficients(0.0068964879028499126);
  model.add_distortion_coefficients(0);
  model.set_frame_name("tracking_camera/front/left");
  return model;
}

}  // namespace farm_ng
