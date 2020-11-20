#include "farm_ng/perception/apriltag.h"

#include <apriltag.h>
#include <apriltag_pose.h>
#include <glog/logging.h>
#include <tag36h11.h>
#include <sophus/se3.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/sophus_protobuf.h"

using farm_ng::core::Bucket;
using farm_ng::core::EventBus;
using farm_ng::core::GetBucketAbsolutePath;
using farm_ng::core::MakeEvent;
using farm_ng::core::ReadProtobufFromJsonFile;

namespace farm_ng {
namespace perception {

std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection) {
  CHECK(detection.tag_size() > 0.0);
  double half_size = detection.tag_size() / 2.0;
  return std::array<Eigen::Vector3d, 4>(
      {Eigen::Vector3d(-half_size, -half_size, 0),
       Eigen::Vector3d(half_size, -half_size, 0.0),
       Eigen::Vector3d(half_size, half_size, 0.0),
       Eigen::Vector3d(-half_size, half_size, 0.0)});
}

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection) {
  return std::array<Eigen::Vector2d, 4>(
      {Eigen::Vector2d(detection.p(0).x(), detection.p(0).y()),
       Eigen::Vector2d(detection.p(1).x(), detection.p(1).y()),
       Eigen::Vector2d(detection.p(2).x(), detection.p(2).y()),
       Eigen::Vector2d(detection.p(3).x(), detection.p(3).y())});
}

double TagSize(const TagLibrary& tag_library, int tag_id) {
  auto it = std::find_if(tag_library.tags().begin(), tag_library.tags().end(),
                         [tag_id](const TagConfig& tag_config) {
                           return tag_config.id() == tag_id;
                         });

  if (it != tag_library.tags().end()) {
    return it->size();
  }
  return 1;
}

// Taken from
// https://github.com/IntelRealSense/librealsense/blob/d8f5d4212df85522a14b4a7b83bf4d54219b06fa/examples/pose-apriltag/rs-pose-apriltag.cpp#L249
// Re-compute homography between ideal standard tag image and undistorted tag
// corners for estimage_tag_pose().
//
// @param[in]  c is 4 pairs of tag corners on ideal image and undistorted input
// image.
// @param[out] H is the output homography between ideal and undistorted input
// image.
// @see        static void apriltag_manager::undistort(...)
//
namespace {
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

void deproject(double* pt_out, const CameraModel& camera_model,
               const double px[2]) {
  // taken from:
  // github.com/IntelRealSense/librealsense/blob/d8f5d4212df85522a14b4a7b83bf4d54219b06fa/examples/pose-apriltag/rs-pose-apriltag.cpp#L166
  Eigen::Map<Eigen::Vector2d> pt(pt_out);
  pt = ReprojectPixelToPoint(camera_model, Eigen::Vector2d(px[0], px[1]), 1.0)
           .head<2>();
}

bool undistort(apriltag_detection_t& src, const CameraModel& camera_model) {
  // Taken from:
  // github.com/IntelRealSense/librealsense/blob/d8f5d4212df85522a14b4a7b83bf4d54219b06fa/examples/pose-apriltag/rs-pose-apriltag.cpp#L149
  deproject(src.c, camera_model, src.c);

  double corr_arr[4][4];
  for (int c = 0; c < 4; ++c) {
    deproject(src.p[c], camera_model, src.p[c]);

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

// https://github.com/IntelRealSense/librealsense/blob/master/examples/pose-apriltag/rs-pose-apriltag.cpp
// Note this function mutates detection, by undistorting the points and
// populating the homography
boost::optional<Sophus::SE3d> EstimateCameraPoseTag(
    const CameraModel& camera_model, const TagLibrary& tag_library,
    apriltag_detection_t* detection) {
  apriltag_detection_info_t info;
  info.fx = info.fy = 1;  // undistorted image with focal length = 1
  info.cx = info.cy = 0;  // undistorted image with principal point at (0,0)
  info.det = detection;
  info.tagsize = TagSize(tag_library, info.det->id);

  // recompute tag corners on an undistorted image focal
  // length = 1 this also populates the homography
  if (!undistort(*info.det, camera_model)) {
    LOG(WARNING) << "Tag with id: " << info.det->id << " can not compute pose.";
    return boost::none;
  }
  auto pose = std::shared_ptr<apriltag_pose_t>(new apriltag_pose_t(),
                                               apriltag_pose_destroy);
  // estimate tag pose in camera coordinate
  estimate_pose_for_tag_homography(&info, pose.get());
  return ApriltagPoseToSE3d(*pose);
}

}  // namespace
class ApriltagDetector::Impl {
 public:
  Impl(const CameraModel& camera_model, EventBus* event_bus)
      : event_bus_(event_bus), camera_model_(camera_model) {
    tag_family_ = std::shared_ptr<apriltag_family_t>(tag36h11_create(),
                                                     &tag36h11_destroy);

    tag_detector_ = std::shared_ptr<apriltag_detector_t>(
        apriltag_detector_create(), &apriltag_detector_destroy);

    apriltag_detector_add_family(tag_detector_.get(), tag_family_.get());
    tag_detector_->quad_decimate = 1.0;
    tag_detector_->quad_sigma = 0.8;
    tag_detector_->nthreads = 1;
    tag_detector_->debug = false;
    tag_detector_->refine_edges = true;
  }

  ApriltagDetections Detect(const cv::Mat& gray,
                            const google::protobuf::Timestamp& stamp) {
    if (!apriltag_config_) {
      LoadApriltagConfig();
    }

    CHECK_EQ(gray.channels(), 1);
    CHECK_EQ(gray.type(), CV_8UC1);

    auto start = std::chrono::high_resolution_clock::now();

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    std::shared_ptr<zarray_t> detections(
        apriltag_detector_detect(tag_detector_.get(), &im),
        apriltag_detections_destroy);

    // copy detections into protobuf
    ApriltagDetections pb_out;
    pb_out.mutable_image()->mutable_camera_model()->CopyFrom(camera_model_);

    for (int i = 0; i < zarray_size(detections.get()); i++) {
      apriltag_detection_t* det;
      zarray_get(detections.get(), i, &det);

      ApriltagDetection* detection = pb_out.add_detections();
      for (int j = 0; j < 4; j++) {
        Vec2* p_j = detection->add_p();
        p_j->set_x(det->p[j][0]);
        p_j->set_y(det->p[j][1]);
      }
      CHECK(apriltag_config_.has_value());
      detection->set_tag_size(
          TagSize(apriltag_config_.value().tag_library(), det->id));
      detection->mutable_c()->set_x(det->c[0]);
      detection->mutable_c()->set_y(det->c[1]);
      detection->set_id(det->id);
      detection->set_hamming(static_cast<uint8_t>(det->hamming));
      detection->set_decision_margin(det->decision_margin);
      // estimation comes last because this mutates det
      auto pose = EstimateCameraPoseTag(
          camera_model_, apriltag_config_.value().tag_library(), det);
      if (pose) {
        auto* named_pose = detection->mutable_pose();
        SophusToProto(*pose, named_pose->mutable_a_pose_b());
        named_pose->mutable_a_pose_b()->mutable_stamp()->CopyFrom(stamp);
        named_pose->set_frame_a(camera_model_.frame_name());
        named_pose->set_frame_b("tag/" + std::to_string(det->id));
        if (event_bus_) {
          event_bus_->Send(MakeEvent("pose/" + camera_model_.frame_name() +
                                         "/tag/" + std::to_string(det->id),
                                     *named_pose, stamp));
        }
      }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    LOG_EVERY_N(INFO, 100) << "april tag detection took: " << duration.count()
                           << " milliseconds\n"
                           << pb_out.ShortDebugString();
    return pb_out;
  }
  void LoadApriltagConfig() {
    apriltag_config_ = ReadProtobufFromJsonFile<ApriltagConfig>(
        GetBucketAbsolutePath(Bucket::BUCKET_CONFIGURATIONS) / "apriltag.json");
    LOG(INFO) << " apriltag config: "
              << apriltag_config_.value().ShortDebugString();
  }

  EventBus* event_bus_;
  CameraModel camera_model_;

  std::optional<ApriltagConfig> apriltag_config_;

  std::shared_ptr<apriltag_family_t> tag_family_;
  std::shared_ptr<apriltag_detector_t> tag_detector_;
};

ApriltagDetector::ApriltagDetector(const CameraModel& camera_model,
                                   EventBus* event_bus)
    : impl_(new Impl(camera_model, event_bus)) {}

ApriltagDetector::~ApriltagDetector() {}

void ApriltagDetector::Close() { impl_->apriltag_config_.reset(); }

ApriltagDetections ApriltagDetector::Detect(
    const cv::Mat& gray, const google::protobuf::Timestamp& stamp) {
  return impl_->Detect(gray, stamp);
}

ApriltagsFilter::ApriltagsFilter() : once_(false) {}
void ApriltagsFilter::Reset() {
  mask_ = cv::Mat();
  once_ = false;
}
bool ApriltagsFilter::AddApriltags(const ApriltagDetections& detections) {
  const int n_tags = detections.detections_size();
  if (n_tags == 0) {
    Reset();
    return false;
  }

  if (mask_.empty()) {
    mask_ =
        cv::Mat::zeros(GetCvSize(detections.image().camera_model()), CV_8UC1);
  }
  CHECK(!mask_.empty());
  cv::Mat new_mask = cv::Mat::zeros(mask_.size(), CV_8UC1);
  const int window_size = 7;
  double mean_count = 0.0;
  cv::Rect mask_roi(0, 0, mask_.size().width, mask_.size().height);
  for (const ApriltagDetection& detection : detections.detections()) {
    for (const auto& p : detection.p()) {
      cv::Rect roi(p.x() - window_size / 2, p.y() - window_size / 2,
                   window_size, window_size);
      roi = roi & mask_roi;
      if (roi.empty()) {
        continue;
      }
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

}  // namespace perception
}  // namespace farm_ng
