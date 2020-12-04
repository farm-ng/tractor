#include "farm_ng/perception/frame_grabber.h"

#include <google/protobuf/util/time_util.h>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>

using farm_ng::core::EventBus;

namespace farm_ng {
namespace perception {

namespace {
// Convert rs2::frame to cv::Mat
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
class FrameGrabberIntel : public FrameGrabber {
 public:
  FrameGrabberIntel(EventBus& event_bus, CameraConfig config)
      : event_bus_(event_bus), config_(config) {
    try {
      rs2::config cfg;
      cfg.enable_device(config_.serial_number());
      if (config_.model() == CameraConfig::MODEL_INTEL_T265) {
        cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
        auto profile = cfg.resolve(pipe_);
        auto tm2 = profile.get_device().as<rs2::tm2>();
        //    auto pose_sensor = tm2.first<rs2::pose_sensor>();

        // Start pipe and get camera calibrations
        const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
        auto fisheye_stream =
            profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
        auto fisheye_intrinsics =
            fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();

        SetCameraModelFromRs(&camera_model_, fisheye_intrinsics);
        camera_model_.set_frame_name(config.name() + "/left");
      } else if (config_.model() == CameraConfig::MODEL_INTEL_D435I) {
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);

        auto profile = cfg.resolve(pipe_);
        auto color_intrinsics = profile.get_stream(RS2_STREAM_COLOR)
                                    .as<rs2::video_stream_profile>()
                                    .get_intrinsics();
        SetCameraModelFromRs(&camera_model_, color_intrinsics);
        camera_model_.set_frame_name(config.name() + "/color");
      } else if (config_.model() == CameraConfig::MODEL_INTEL_D415) {
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);

        auto profile = cfg.resolve(pipe_);

        auto color_intrinsics = profile.get_stream(RS2_STREAM_COLOR)
                                    .as<rs2::video_stream_profile>()
                                    .get_intrinsics();
        SetCameraModelFromRs(&camera_model_, color_intrinsics);
        camera_model_.set_frame_name(config.name() + "/color");
      }
      /// detector_.reset(new ApriltagDetector(camera_model_, &event_bus_));

      LOG(INFO) << " intrinsics model: " << camera_model_.ShortDebugString();
      // frame_video_writer_ = std::make_unique<VideoStreamer>(
      // event_bus_, camera_model_, VideoStreamer::MODE_MP4_FILE);

      if (config_.has_udp_stream_port()) {
        // udp_streamer_ = std::make_unique<VideoStreamer>(
        // event_bus_, camera_model_, VideoStreamer::MODE_MP4_UDP,
        // config_.udp_stream_port().value());
      }

      pipe_.start(cfg, std::bind(&FrameGrabberIntel::frame_callback, this,
                                 std::placeholders::_1));
    } catch (const rs2::error& e) {
      std::stringstream ss;
      ss << "RealSense error calling " << e.get_failed_function() << "("
         << e.get_failed_args() << "):\n    " << e.what();
      LOG(ERROR) << ss.str();
      throw std::runtime_error(ss.str());
    }
  }

  // The callback is executed on a sensor thread and can be called
  // simultaneously from multiple sensors Therefore any modification to common
  // memory should be done under lock
  void frame_callback(const rs2::frame& frame) {
    VLOG(1) << "frame recved : " << camera_model_.frame_name();
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      std::optional<rs2::video_frame> video_frame;
      if (config_.model() == CameraConfig::MODEL_INTEL_D435I) {
        video_frame = fs.get_color_frame();
      } else if (config_.model() == CameraConfig::MODEL_INTEL_D415) {
        video_frame = fs.get_color_frame();
      } else if (config_.model() == CameraConfig::MODEL_INTEL_T265) {
        video_frame = fs.get_fisheye_frame(0);
      }
      cv::Mat frame_0 = RS2FrameToMat(*video_frame).clone();
      auto stamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          video_frame->get_timestamp());

      std::lock_guard<std::mutex> lock(mtx_);
      signal_(FrameData({config_, camera_model_, frame_0, stamp}));
    }
  }
  virtual ~FrameGrabberIntel() = default;

  virtual const CameraConfig& GetCameraConfig() const override {
    return config_;
  }
  virtual const CameraModel& GetCameraModel() const override {
    return camera_model_;
  };

  virtual FrameGrabber::Signal& VisualFrameSignal() override { return signal_; }

  EventBus& event_bus_;

  rs2::pipeline pipe_;
  CameraConfig config_;
  CameraModel camera_model_;
  FrameGrabber::Signal signal_;
  std::mutex mtx_;
};  // namespace farm_ng

namespace {
static FrameGrabber::FrameGrabberFactory intel_factory =
    [](EventBus& event_bus, CameraConfig config) {
      return std::unique_ptr<FrameGrabber>(
          new FrameGrabberIntel(event_bus, config));
    };

static int _t265 = FrameGrabber::AddFrameGrabberFactory(
    CameraConfig::Model_Name(CameraConfig::MODEL_INTEL_T265), intel_factory);
static int _435i = FrameGrabber::AddFrameGrabberFactory(
    CameraConfig::Model_Name(CameraConfig::MODEL_INTEL_D435I), intel_factory);
static int _t435i = FrameGrabber::AddFrameGrabberFactory(
    CameraConfig::Model_Name(CameraConfig::MODEL_INTEL_D415), intel_factory);
}  // namespace

}  // namespace perception
}  // namespace farm_ng
