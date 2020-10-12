#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/util/time_util.h>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"

#include <farm_ng/init.h>
#include <farm_ng/ipc.h>
#include <farm_ng/sophus_protobuf.h>

#include <farm_ng_proto/tractor/v1/apriltag.pb.h>
#include <farm_ng_proto/tractor/v1/geometry.pb.h>
#include <farm_ng_proto/tractor/v1/tracking_camera.pb.h>

typedef farm_ng_proto::tractor::v1::Event EventPb;

using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::Vec2;

using farm_ng_proto::tractor::v1::ApriltagConfig;
using farm_ng_proto::tractor::v1::ApriltagDetection;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::BUCKET_CONFIGURATIONS;
using farm_ng_proto::tractor::v1::TagConfig;
using farm_ng_proto::tractor::v1::TagLibrary;

using farm_ng_proto::tractor::v1::CameraModel;
using farm_ng_proto::tractor::v1::TrackingCameraCommand;
using farm_ng_proto::tractor::v1::TrackingCameraPoseFrame;

DEFINE_bool(jetson, false, "Use jetson hardware encoding.");

namespace farm_ng {

struct ScopeGuard {
  ScopeGuard(std::function<void()> function) : function_(std::move(function)) {}
  ~ScopeGuard() { function_(); }
  std::function<void()> function_;
};

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

void SetVec3FromRs(farm_ng_proto::tractor::v1::Vec3* out, rs2_vector vec) {
  out->set_x(vec.x);
  out->set_y(vec.y);
  out->set_z(vec.z);
}

void SetQuatFromRs(farm_ng_proto::tractor::v1::Quaternion* out,
                   rs2_quaternion vec) {
  out->set_w(vec.w);
  out->set_x(vec.x);
  out->set_y(vec.y);
  out->set_z(vec.z);
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
    default:
      CHECK(false) << "Unhandled intrinsics model: "
                   << rs2_distortion_to_string(intrinsics.model);
  }
  for (int i = 0; i < 5; ++i) {
    out->add_distortion_coefficients(intrinsics.coeffs[i]);
  }
}

TrackingCameraPoseFrame::Confidence ToConfidence(int x) {
  switch (x) {
    case 0:
      return TrackingCameraPoseFrame::CONFIDENCE_FAILED;
    case 1:
      return TrackingCameraPoseFrame::CONFIDENCE_LOW;
    case 2:
      return TrackingCameraPoseFrame::CONFIDENCE_MEDIUM;
    case 3:
      return TrackingCameraPoseFrame::CONFIDENCE_HIGH;
    default:
      return TrackingCameraPoseFrame::CONFIDENCE_UNSPECIFIED;
  }
}

TrackingCameraPoseFrame ToPoseFrame(const rs2::pose_frame& rs_pose_frame) {
  TrackingCameraPoseFrame pose_frame;
  pose_frame.set_frame_number(rs_pose_frame.get_frame_number());
  *pose_frame.mutable_stamp_pose() =
      google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          rs_pose_frame.get_timestamp());
  auto pose_data = rs_pose_frame.get_pose_data();
  SetVec3FromRs(pose_frame.mutable_start_pose_current()->mutable_position(),
                pose_data.translation);
  SetQuatFromRs(pose_frame.mutable_start_pose_current()->mutable_rotation(),
                pose_data.rotation);
  SetVec3FromRs(pose_frame.mutable_velocity(), pose_data.velocity);
  SetVec3FromRs(pose_frame.mutable_acceleration(), pose_data.acceleration);
  SetVec3FromRs(pose_frame.mutable_angular_velocity(),
                pose_data.angular_velocity);
  SetVec3FromRs(pose_frame.mutable_angular_acceleration(),
                pose_data.angular_acceleration);
  pose_frame.set_tracker_confidence(ToConfidence(pose_data.tracker_confidence));
  pose_frame.set_mapper_confidence(ToConfidence(pose_data.mapper_confidence));
  return pose_frame;
}

EventPb ToNamedPoseEvent(const rs2::pose_frame& rs_pose_frame) {
  // TODO(ethanrublee) support front and rear cameras.
  NamedSE3Pose vodom_pose_t265;
  // here we distinguish where visual_odom frame by which camera it refers to,
  // will have to connect each camera pose to the mobile base with an extrinsic
  // transform
  vodom_pose_t265.set_frame_b("odometry/tracking_camera/front");
  vodom_pose_t265.set_frame_a("tracking_camera/front");
  auto pose_data = rs_pose_frame.get_pose_data();
  SetVec3FromRs(vodom_pose_t265.mutable_a_pose_b()->mutable_position(),
                pose_data.translation);
  SetQuatFromRs(vodom_pose_t265.mutable_a_pose_b()->mutable_rotation(),
                pose_data.rotation);

  Sophus::SE3d se3;
  ProtoToSophus(vodom_pose_t265.a_pose_b(), &se3);
  SophusToProto(se3.inverse(), vodom_pose_t265.mutable_a_pose_b());

  EventPb event =
      farm_ng::MakeEvent("pose/tracking_camera/front", vodom_pose_t265);
  *event.mutable_stamp() =
      google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          rs_pose_frame.get_timestamp());
  return event;
}

//
// Re-compute homography between ideal standard tag image and undistorted tag
// corners for estimage_tag_pose().
//
// @param[in]  c is 4 pairs of tag corners on ideal image and undistorted input
// image.
// @param[out] H is the output homography between ideal and undistorted input
// image.
// @see        static void apriltag_manager::undistort(...)
//
bool ComputeHomography(const double c[4][4], matd_t* H) {
  double A[] = {
      c[0][0],
      c[0][1],
      1,
      0,
      0,
      0,
      -c[0][0] * c[0][2],
      -c[0][1] * c[0][2],
      c[0][2],
      0,
      0,
      0,
      c[0][0],
      c[0][1],
      1,
      -c[0][0] * c[0][3],
      -c[0][1] * c[0][3],
      c[0][3],
      c[1][0],
      c[1][1],
      1,
      0,
      0,
      0,
      -c[1][0] * c[1][2],
      -c[1][1] * c[1][2],
      c[1][2],
      0,
      0,
      0,
      c[1][0],
      c[1][1],
      1,
      -c[1][0] * c[1][3],
      -c[1][1] * c[1][3],
      c[1][3],
      c[2][0],
      c[2][1],
      1,
      0,
      0,
      0,
      -c[2][0] * c[2][2],
      -c[2][1] * c[2][2],
      c[2][2],
      0,
      0,
      0,
      c[2][0],
      c[2][1],
      1,
      -c[2][0] * c[2][3],
      -c[2][1] * c[2][3],
      c[2][3],
      c[3][0],
      c[3][1],
      1,
      0,
      0,
      0,
      -c[3][0] * c[3][2],
      -c[3][1] * c[3][2],
      c[3][2],
      0,
      0,
      0,
      c[3][0],
      c[3][1],
      1,
      -c[3][0] * c[3][3],
      -c[3][1] * c[3][3],
      c[3][3],
  };

  double epsilon = 1e-10;

  // Eliminate.
  for (int col = 0; col < 8; col++) {
    // Find best row to swap with.
    double max_val = 0;
    int max_val_idx = -1;
    for (int row = col; row < 8; row++) {
      double val = fabs(A[row * 9 + col]);
      if (val > max_val) {
        max_val = val;
        max_val_idx = row;
      }
    }

    if (max_val < epsilon) {
      return false;
    }

    // Swap to get best row.
    if (max_val_idx != col) {
      for (int i = col; i < 9; i++) {
        double tmp = A[col * 9 + i];
        A[col * 9 + i] = A[max_val_idx * 9 + i];
        A[max_val_idx * 9 + i] = tmp;
      }
    }

    // Do eliminate.
    for (int i = col + 1; i < 8; i++) {
      double f = A[i * 9 + col] / A[col * 9 + col];
      A[i * 9 + col] = 0;
      for (int j = col + 1; j < 9; j++) {
        A[i * 9 + j] -= f * A[col * 9 + j];
      }
    }
  }

  // Back solve.
  for (int col = 7; col >= 0; col--) {
    double sum = 0;
    for (int i = col + 1; i < 8; i++) {
      sum += A[col * 9 + i] * A[i * 9 + 8];
    }
    A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
  }
  H->data[0] = A[8];
  H->data[1] = A[17];
  H->data[2] = A[26];
  H->data[3] = A[35];
  H->data[4] = A[44];
  H->data[5] = A[53];
  H->data[6] = A[62];
  H->data[7] = A[71];
  H->data[8] = 1;
  return true;
}

void deproject(double pt[2], const rs2_intrinsics& intr, const double px[2]) {
  float fpt[3], fpx[2] = {(float)px[0], (float)px[1]};
  rs2_deproject_pixel_to_point(fpt, &intr, fpx, 1.0f);
  pt[0] = fpt[0];
  pt[1] = fpt[1];
}

bool undistort(apriltag_detection_t& src, const rs2_intrinsics& intr) {
  deproject(src.c, intr, src.c);

  double corr_arr[4][4];
  for (int c = 0; c < 4; ++c) {
    deproject(src.p[c], intr, src.p[c]);

    corr_arr[c][0] =
        (c == 0 || c == 3) ? -1 : 1;  // tag corners in an ideal image
    corr_arr[c][1] =
        (c == 0 || c == 1) ? -1 : 1;  // tag corners in an ideal image
    corr_arr[c][2] =
        src.p[c][0];  // tag corners in undistorted image focal length = 1
    corr_arr[c][3] =
        src.p[c][1];  // tag corners in undistorted image focal length = 1
  }
  if (src.H == nullptr) {
    src.H = matd_create(3, 3);
  }
  return ComputeHomography(corr_arr, src.H);
}

void apriltag_pose_destroy(apriltag_pose_t* p) {
  matd_destroy(p->R);
  matd_destroy(p->t);
  delete p;
}

Sophus::SE3d ApriltagPoseToSE3d(const apriltag_pose_t& pose) {
  typedef Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
      Map33RowMajor;
  return Sophus::SE3d(
      Map33RowMajor(pose.R->data),
      Eigen::Vector3d(pose.t->data[0], pose.t->data[1], pose.t->data[2]));
}

double TagSize(const TagLibrary& tag_library, int tag_id) {
  auto it = std::find_if(tag_library.tags().begin(), tag_library.tags().end(),
                         [tag_id](const TagConfig& tag_config) {
                           return tag_config.id() == tag_id;
                         });

  if (it != tag_library.tags().end()) {
    return it->size();
  }
  return 0;
}

// https://github.com/IntelRealSense/librealsense/blob/master/examples/pose-apriltag/rs-pose-apriltag.cpp
// Note this function mutates detection, by undistorting the points and
// populating the homography
boost::optional<Sophus::SE3d> EstimateCameraPoseTag(
    const rs2_intrinsics& intr, const TagLibrary& tag_library,
    apriltag_detection_t* detection) {
  apriltag_detection_info_t info;
  info.fx = info.fy = 1;  // undistorted image with focal length = 1
  info.cx = info.cy = 0;  // undistorted image with principal point at (0,0)
  info.det = detection;
  info.tagsize = TagSize(tag_library, info.det->id);

  // recompute tag corners on an undistorted image focal
  // length = 1 this also populates the homography
  if (!undistort(*info.det, intr)) {
    LOG(WARNING) << "Tag with id: " << info.det->id << " can not compute pose.";
    return boost::none;
  }
  auto pose = std::shared_ptr<apriltag_pose_t>(new apriltag_pose_t(),
                                               apriltag_pose_destroy);
  // estimate tag pose in camera coordinate
  estimate_pose_for_tag_homography(&info, pose.get());
  return ApriltagPoseToSE3d(*pose);
}

class ApriltagDetector {
  // see
  // https://github.com/AprilRobotics/apriltag/blob/master/example/opencv_demo.cc
 public:
  ApriltagDetector(CameraModel camera_model, const TagLibrary& tag_library,
                   EventBus* event_bus = nullptr)
      : event_bus_(event_bus),
        tag_library_(tag_library),
        camera_model_(camera_model) {
    tag_family_ = tag36h11_create();
    tag_detector_ = apriltag_detector_create();
    apriltag_detector_add_family(tag_detector_, tag_family_);
    tag_detector_->quad_decimate = 1.0;
    tag_detector_->quad_sigma = 0.8;
    tag_detector_->nthreads = 1;
    tag_detector_->debug = false;
    tag_detector_->refine_edges = true;
  }

  ~ApriltagDetector() {
    apriltag_detector_destroy(tag_detector_);
    tag36h11_destroy(tag_family_);
  }

  ApriltagDetections Detect(cv::Mat gray, google::protobuf::Timestamp stamp) {
    CHECK_EQ(gray.channels(), 1);
    CHECK_EQ(gray.type(), CV_8UC1);

    auto start = std::chrono::high_resolution_clock::now();

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    std::shared_ptr<zarray_t> detections(
        apriltag_detector_detect(tag_detector_, &im),
        apriltag_detections_destroy);

    // copy detections into protobuf
    ApriltagDetections pb_out;
    pb_out.mutable_image()->mutable_camera_model()->CopyFrom(camera_model_);

    for (int i = 0; i < zarray_size(detections.get()); i++) {
      apriltag_detection_t* det;
      zarray_get(detections.get(), i, &det);
      int tag_size = TagSize(tag_library_, det->id);

      ApriltagDetection* detection = pb_out.add_detections();
      for (int j = 0; j < 4; j++) {
        Vec2* p_j = detection->add_p();
        p_j->set_x(det->p[j][0]);
        p_j->set_y(det->p[j][1]);
      }
      detection->set_tag_size(tag_size);
      detection->mutable_c()->set_x(det->c[0]);
      detection->mutable_c()->set_y(det->c[1]);
      detection->set_id(det->id);
      detection->set_hamming(static_cast<uint8_t>(det->hamming));
      detection->set_decision_margin(det->decision_margin);
      // estimation comes last because this mutates det
      auto pose = EstimateCameraPoseTag(intrinsics_, tag_library_, det);
      if (pose) {
        auto* named_pose = detection->mutable_pose();
        SophusToProto(*pose, named_pose->mutable_a_pose_b());
        named_pose->mutable_a_pose_b()->mutable_stamp()->CopyFrom(stamp);
        named_pose->set_frame_a("tracking_camera/front/left");
        named_pose->set_frame_b("tag/" + std::to_string(det->id));
        if (event_bus_) {
          event_bus_->Send(MakeEvent(
              "pose/tracking_camera/front/left/tag/" + std::to_string(det->id),
              *named_pose, stamp));
        }
      }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    LOG_EVERY_N(INFO, 100) << "april tag detection took: " << duration.count()
                           << " microseconds\n"
                           << pb_out.ShortDebugString();
    return pb_out;
  }

 private:
  EventBus* event_bus_;
  rs2_intrinsics intrinsics_;
  TagLibrary tag_library_;
  CameraModel camera_model_;
  apriltag_family_t* tag_family_;
  apriltag_detector_t* tag_detector_;
};

cv::Size GetCvSize(const CameraModel& model) {
  return cv::Size(model.image_width(), model.image_height());
}

// This class is meant to help filter apriltags, returning true once after the
// camera becomes relatively stationary.  To allow for a capture program which
// automatically captures a sparse set of unique view points for calibration
// after person or robot moves to position and stops for some reasonable period
// and then continues. It uses a simple 2d accumulated mask based hueristic.
//
//   1. for each detected apriltag point, define a small ROI (e.g. 7x7)
//       1. Construct a new mask the size of the detection image,
//          which is set to 0 everywhere except for in the ROI
//            mask = 0
//            mask(ROI) = previous_mask(ROI) + 1
//       2. Find the max count in the ROI, this is essentially the number of
//       frames where this tag point has kept roughly within the ROI.
//   2. Compute the mean of the max counts.
//   3. If the mean is moves above a threshold, determined experimentally to
//   mean stable, 5 at my desk lab, then this detection is considered stable. If
//   the camera stays still, we're only interested in the transition, because we
//   don't want duplicate frames.
//
//   NOTE If the camera moves slowly but constantly, this tends to capture every
//   other frame.
class ApriltagsFilter {
 public:
  ApriltagsFilter() : once_(false) {}
  void Reset() {
    mask_ = cv::Mat();
    once_ = false;
  }
  bool AddApriltags(const ApriltagDetections& detections) {
    const int n_tags = detections.detections_size();
    if (n_tags == 0) {
      Reset();
      return false;
    }

    if (mask_.empty()) {
      mask_ =
          cv::Mat::zeros(GetCvSize(detections.image().camera_model()), CV_8UC1);
    }

    cv::Mat new_mask = cv::Mat::zeros(mask_.size(), CV_8UC1);
    const int window_size = 7;
    double mean_count = 0.0;
    for (const ApriltagDetection& detection : detections.detections()) {
      for (const auto& p : detection.p()) {
        cv::Rect roi(p.x() - window_size / 2, p.y() - window_size / 2,
                     window_size, window_size);
        new_mask(roi) = mask_(roi) + 1;
        double max_val = 0.0;
        cv::minMaxLoc(new_mask(roi), nullptr, &max_val);
        mean_count += max_val / (4 * n_tags);
      }
    }
    mask_ = new_mask;
    const int kThresh = 5;
    if (mean_count > kThresh && !once_) {
      once_ = true;
      return true;
    }
    if (mean_count < kThresh) {
      once_ = false;
    }
    return false;
  }

 private:
  cv::Mat mask_;
  bool once_;
};

class VideoFileWriter {
 public:
  VideoFileWriter(EventBus& bus, CameraModel camera_model) : bus_(bus) {
    image_pb_.mutable_camera_model()->CopyFrom(camera_model);
    image_pb_.mutable_fps()->set_value(30);
    image_pb_.mutable_frame_number()->set_value(0);
  }

  void ResetVideoWriter() {
    std::string encoder_x264(" x264enc ! ");
    std::string encoder_omxh264(" omxh264enc bitrate=10000000 ! ");

    auto resource_path = GetUniqueArchiveResource(
        image_pb_.camera_model().frame_name(), "mp4", "video/mp4");
    std::string video_writer_dev =
        std::string("appsrc !") + " videoconvert ! " +
        (FLAGS_jetson ? encoder_omxh264 : encoder_x264) +
        " mp4mux ! filesink location=" + resource_path.second.string();
    LOG(INFO) << "writing video with: " << video_writer_dev;
    writer_ = std::make_shared<cv::VideoWriter>(
        video_writer_dev,
        0,                        // fourcc
        image_pb_.fps().value(),  // fps
        cv::Size(image_pb_.camera_model().image_width(),
                 image_pb_.camera_model().image_height()),
        false  // isColor
    );
    image_pb_.mutable_resource()->CopyFrom(resource_path.first);
    image_pb_.mutable_frame_number()->set_value(0);
  }
  void AddFrame(cv::Mat image, google::protobuf::Timestamp stamp) {
    if (!writer_) {
      ResetVideoWriter();
    }
    writer_->write(image);
    image_pb_.mutable_frame_number()->set_value(
        image_pb_.frame_number().value() + 1);
    bus_.Send(MakeEvent(image_pb_.camera_model().frame_name() + "/image",
                        image_pb_, stamp));

    if (image_pb_.frame_number().value() >= k_max_frames_) {
      writer_.reset();
    }
  }

  void Close() { writer_.reset(); }

 private:
  EventBus& bus_;
  Image image_pb_;
  std::shared_ptr<cv::VideoWriter> writer_;
  const uint k_max_frames_ = 300;
};

class TrackingCameraClient {
 public:
  TrackingCameraClient(EventBus& bus)
      : io_service_(bus.get_io_service()), event_bus_(bus) {
    event_bus_.GetEventSignal()->connect(std::bind(
        &TrackingCameraClient::on_event, this, std::placeholders::_1));
    // TODO(ethanrublee) look up image size from realsense profile.

    ApriltagConfig apriltag_config = ReadProtobufFromJsonFile<ApriltagConfig>(
        GetBucketAbsolutePath(BUCKET_CONFIGURATIONS) / "apriltag.config");

    std::string encoder_x264 =
        std::string(
            " x264enc bitrate=600 speed-preset=ultrafast tune=zerolatency ") +
        "key-int-max=15 ! video/x-h264,profile=constrained-baseline ! queue " +
        "max-size-time=100000000 ! h264parse ! ";

    std::string encoder_omxh264 =
        std::string(" omxh264enc control-rate=1 bitrate=1000000 ! ") +
        " video/x-h264, stream-format=byte-stream !";

    std::string cmd0 = std::string("appsrc !") + " videoconvert ! " +
                       (FLAGS_jetson ? encoder_omxh264 : encoder_x264) +

                       " rtph264pay pt=96 mtu=1400 config-interval=10 !" +
                       " udpsink port=5000";
    std::cerr << "Running gstreamer with pipeline:\n" << cmd0 << std::endl;
    std::cerr << "To view streamer run:\n"
              << "gst-launch-1.0 udpsrc port=5000 "
                 "! application/x-rtp,encoding-name=H264,payload=96 ! "
                 "rtph264depay ! h264parse ! queue ! avdec_h264 ! xvimagesink "
                 "sync=false async=false -e"
              << std::endl;

    writer_.reset(new cv::VideoWriter(cmd0,
                                      0,   // fourcc
                                      10,  // fps
                                      cv::Size(848, 800),
                                      false  // isColor
                                      ));

    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    auto profile = cfg.resolve(pipe_);
    auto tm2 = profile.get_device().as<rs2::tm2>();
    auto pose_sensor = tm2.first<rs2::pose_sensor>();

    // Start pipe and get camera calibrations
    const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
    auto fisheye_stream =
        profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    auto fisheye_intrinsics =
        fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();

    SetCameraModelFromRs(&left_camera_model_, fisheye_intrinsics);
    left_camera_model_.set_frame_name(
        "tracking_camera/front/left");  // TODO Pass in constructor.
    detector_.reset(new ApriltagDetector(
        left_camera_model_, apriltag_config.tag_library(), &event_bus_));
    LOG(INFO) << " intrinsics model: " << left_camera_model_.ShortDebugString();
    frame_video_writer_ =
        std::make_unique<VideoFileWriter>(event_bus_, left_camera_model_);
    // setting options for slam:
    // https://github.com/IntelRealSense/librealsense/issues/1011
    // and what to set:
    //  https://github.com/IntelRealSense/realsense-ros/issues/779 "
    // I would suggest leaving mapping enabled, but disabling
    // relocalization and jumping. This may avoid the conflict with
    // RTabMap while still giving good results."

    pose_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 0);
    pose_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 0);

    // Start pipeline with chosen configuration
    pipe_.start(cfg, std::bind(&TrackingCameraClient::frame_callback, this,
                               std::placeholders::_1));
  }

  void on_command(const TrackingCameraCommand& command) {
    latest_command_ = command;
    LOG(INFO) << "Got command: " << latest_command_.ShortDebugString();
  }

  void on_event(const EventPb& event) {
    if (event.data().type_url() ==
        "type.googleapis.com/" +
            TrackingCameraCommand::descriptor()->full_name()) {
      TrackingCameraCommand command;
      CHECK(event.data().UnpackTo(&command));
      on_command(command);
    }
  }

  void record_every_frame(cv::Mat image, google::protobuf::Timestamp stamp) {
    frame_video_writer_->AddFrame(image, stamp);
  }

  void detect_apriltags(cv::Mat image, google::protobuf::Timestamp stamp) {
    auto apriltags = detector_->Detect(image, stamp);

    if (tag_filter_.AddApriltags(apriltags)) {
      auto resource_path = GetUniqueArchiveResource(
          "tracking_camera_left_apriltag", "png", "image/png");
      apriltags.mutable_image()->mutable_resource()->CopyFrom(
          resource_path.first);

      LOG(INFO) << "Writing to : " << resource_path.second;
      CHECK(cv::imwrite(resource_path.second.string(), image))
          << "Failed to write image to: " << resource_path.second;

      event_bus_.Send(farm_ng::MakeEvent(
          "calibrator/tracking_camera/front/apriltags", apriltags, stamp));
    }

    event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/apriltags",
                                       apriltags, stamp));
  }

  // The callback is executed on a sensor thread and can be called
  // simultaneously from multiple sensors Therefore any modification to common
  // memory should be done under lock
  void frame_callback(const rs2::frame& frame) {
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      rs2::video_frame fisheye_frame = fs.get_fisheye_frame(0);
      // Add a reference to fisheye_frame, cause we're scheduling
      // april tag detection for later.
      fisheye_frame.keep();
      cv::Mat frame_0 = RS2FrameToMat(fisheye_frame);

      // lock for rest of scope, so we can edit some member state.
      std::lock_guard<std::mutex> lock(mtx_realsense_state_);
      count_ = (count_ + 1) % 3;
      if (count_ == 0) {
        writer_->write(frame_0);
      }
      // we only want to schedule detection if we're not currently
      // detecting.  Apriltag detection takes >30ms on the nano.
      if (!detection_in_progress_) {
        detection_in_progress_ = true;

        auto stamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
            fisheye_frame.get_timestamp());

        // schedule april tag detection, do it as frequently as possible.
        io_service_.post([this, fisheye_frame, stamp] {
          ScopeGuard guard([this] {
            // signal that we're done detecting, so can post another frame for
            // detection.
            std::lock_guard<std::mutex> lock(mtx_realsense_state_);
            detection_in_progress_ = false;
          });

          if (!latest_command_.has_record_start()) {
            // Close may be called regardless of state.  If we were recording,
            // it closes the video file on the last chunk.
            frame_video_writer_->Close();
            return;
          }
          // note this function is called later, in main thread, via
          // io_service_.run();
          cv::Mat frame_0 = RS2FrameToMat(fisheye_frame);
          switch (latest_command_.record_start().mode()) {
            case TrackingCameraCommand::RecordStart::MODE_EVERY_FRAME:
              record_every_frame(frame_0, stamp);
              break;
            case TrackingCameraCommand::RecordStart::MODE_APRILTAG_STABLE:
              detect_apriltags(frame_0, stamp);
              break;
            default:
              break;
          }
        });
      }
    }
    /*else if (rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>()) {
      // HACK disable for now, this harms the ipc perfomance.

       auto pose_data = pose_frame.get_pose_data();
      // Print the x, y, z values of the translation, relative to initial
      // position
      // std::cout << "\r Device Position: " << std::setprecision(3) <<
      // std::fixed << pose_data.translation.x << " " <<
     pose_data.translation.y
      // << " " << pose_data.translation.z << " (meters)";
      event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/pose",
                                         ToPoseFrame(pose_frame)));
      event_bus_.Send(ToNamedPoseEvent(pose_frame));
     }*/
  }
  boost::asio::io_service& io_service_;
  EventBus& event_bus_;
  std::unique_ptr<cv::VideoWriter> writer_;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe_;
  std::mutex mtx_realsense_state_;
  int count_ = 0;
  bool detection_in_progress_ = false;
  std::unique_ptr<ApriltagDetector> detector_;
  TrackingCameraCommand latest_command_;
  ApriltagsFilter tag_filter_;
  std::unique_ptr<VideoFileWriter> frame_video_writer_;
  CameraModel left_camera_model_;
};
}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) {}

int Main(farm_ng::EventBus& bus) {
  farm_ng::TrackingCameraClient client(bus);
  try {
    bus.get_io_service().run();
  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
