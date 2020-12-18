#include "farm_ng/perception/frame_grabber.h"

#include <google/protobuf/util/time_util.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <k4a/k4a.hpp>
#include <thread>

#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/camera_model.h"

using farm_ng::core::EventBus;
using farm_ng::core::MakeTimestampNow;

namespace farm_ng {
namespace perception {

class FrameGrabberK4a : public FrameGrabber {
 public:
  FrameGrabberK4a(EventBus& event_bus, CameraConfig config)
      : event_bus_(event_bus), config_(config) {
    try {
      // Check for devices
      //
      const uint32_t deviceCount = k4a::device::get_installed_count();
      if (deviceCount == 0) {
        throw std::runtime_error("No Azure Kinect devices detected!");
      }

      for (uint32_t i = 0; i < deviceCount; ++i) {
        k4a::device dev = k4a::device::open(i);
        std::string serial_number = dev.get_serialnum();
        LOG(INFO) << "k4a::device : " << i << " " << serial_number;
        if (config.serial_number().empty() ||
            serial_number == config.serial_number()) {
          dev_ = std::move(dev);
          break;
        }
      }
      CHECK(dev_) << "No k4a device found matching serial number: "
                  << config.serial_number();

      // Start the device
      k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
      config.camera_fps = K4A_FRAMES_PER_SECOND_30;
      config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
      config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
      config.color_resolution = K4A_COLOR_RESOLUTION_1080P;

      // This means that we'll only get captures that have both color and
      // depth images, so we don't need to check if the capture contains
      // a particular type of image.
      //
      config.synchronized_images_only = true;
      k4a::calibration calibration =
          dev_->get_calibration(config.depth_mode, config.color_resolution);
      CameraModel model;
      auto params =
          calibration.color_camera_calibration.intrinsics.parameters.param;
      model.set_image_width(
          calibration.color_camera_calibration.resolution_width);
      model.set_image_height(
          calibration.color_camera_calibration.resolution_height);
      model.set_cx(params.cx);
      model.set_cy(params.cy);
      model.set_fx(params.fx);
      model.set_fy(params.fy);

      model.set_distortion_model(CameraModel::DISTORTION_MODEL_BROWN_CONRADY);
      model.add_distortion_coefficients(params.k1);
      model.add_distortion_coefficients(params.k2);
      model.add_distortion_coefficients(params.p1);
      model.add_distortion_coefficients(params.p2);
      model.add_distortion_coefficients(params.k3);
      model.add_distortion_coefficients(params.k4);
      model.add_distortion_coefficients(params.k5);
      model.add_distortion_coefficients(params.k6);
      model.set_frame_name(config_.name() + "/color");
      LOG(INFO) << model.ShortDebugString();
      frame_data_.camera_model.CopyFrom(model);
      frame_data_.config.CopyFrom(config_);
      camera_model_.CopyFrom(model);

      capture_thread_.emplace([this, config, calibration]() {
        std::cout << "Started  K4A device..." << std::endl;
        k4a::transformation depth_to_color(calibration);
        dev_->start_cameras(&config);
        while (true) {
          // Poll the device for new image data.
          //
          // We set the timeout to 0 so we don't block if there isn't an
          // available frame.
          //
          // This works here because we're doing the work on the same thread
          // that we're using for the UI, and the ViewerWindow class caps the
          // framerate at the display's refresh rate (the EndFrame() call blocks
          // on the display's sync).
          //
          // If we don't have new image data, we'll just reuse the textures we
          // generated from the last time we got a capture.
          //
          k4a::capture capture;
          if (dev_->get_capture(&capture, std::chrono::milliseconds(0))) {
            auto stamp = MakeTimestampNow();


            const k4a::image colorImage = capture.get_color_image();
            CHECK_EQ(colorImage.get_format(), K4A_IMAGE_FORMAT_COLOR_BGRA32);

                    cv::Mat color_mat(
                colorImage.get_height_pixels(), colorImage.get_width_pixels(),
                CV_8UC4, (void*)colorImage.get_buffer(), cv::Mat::AUTO_STEP);

                const k4a::image depthImage =
                depth_to_color.depth_image_to_color_camera(capture.get_depth_image());
            CHECK_EQ(depthImage.get_format(), K4A_IMAGE_FORMAT_DEPTH16);
             cv::Mat depthmap(
                depthImage.get_height_pixels(), depthImage.get_width_pixels(),
                CV_16UC1, (void*)depthImage.get_buffer(), cv::Mat::AUTO_STEP);


            std::lock_guard<std::mutex> lock(mtx_);

            cv::cvtColor(color_mat, frame_data_.image, cv::COLOR_BGRA2BGR);
            frame_data_.depthmap_range = Depthmap::RANGE_MM;
            frame_data_.depthmap = depthmap.clone();
            frame_data_.mutable_stamp()->CopyFrom(stamp);

            signal_(frame_data_);
            ;
          }
        }
      });

      std::cout << "Finished opening K4A device." << std::endl;

    } catch (const std::exception& e) {
      LOG(FATAL) << e.what();
    }
  }

  virtual ~FrameGrabberK4a() = default;

  virtual const CameraConfig& GetCameraConfig() const override {
    return config_;
  }
  virtual const CameraModel& GetCameraModel() const override {
    return camera_model_;
  };

  virtual FrameGrabber::Signal& VisualFrameSignal() override { return signal_; }

  EventBus& event_bus_;

  CameraConfig config_;
  CameraModel camera_model_;
  FrameGrabber::Signal signal_;
  std::mutex mtx_;
  std::optional<k4a::device> dev_;
  std::optional<std::thread> capture_thread_;
  FrameData frame_data_;
};  // namespace farm_ng

namespace {
static FrameGrabber::FrameGrabberFactory k4a_factory = [](EventBus& event_bus,
                                                          CameraConfig config) {
  return std::unique_ptr<FrameGrabber>(new FrameGrabberK4a(event_bus, config));
};

static int _k4a = FrameGrabber::AddFrameGrabberFactory(
    CameraConfig::Model_Name(CameraConfig::MODEL_K4A), k4a_factory);

}  // namespace

}  // namespace perception
}  // namespace farm_ng
