/*
tractor/build/cpp/farm_ng/tracking_study \
 --base_to_camera_result \
 tractor-data/base_to_camera_models/base_to_camera.json \
 --video_dataset_result \
 tractor-data/video_datasets/tractor0003_20201011_field_loop_01_notags.json \
  --zero_indexed=False
 */
#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/event_log_reader.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

#include "farm_ng/calibration/base_to_camera_calibrator.h"
#include "farm_ng/calibration/camera_model.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/time_series.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrate_apriltag_rig.pb.h"
#include "farm_ng_proto/tractor/v1/calibrate_base_to_camera.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/capture_calibration_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/tractor.pb.h"

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::BaseToCameraModel;
using farm_ng_proto::tractor::v1::CalibrateApriltagRigResult;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraConfiguration;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraResult;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraStatus;
using farm_ng_proto::tractor::v1::TractorState;

using farm_ng_proto::tractor::v1::CaptureCalibrationDatasetResult;
using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::Resource;

using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::SolverStatus;
using farm_ng_proto::tractor::v1::SolverStatus_Name;
using farm_ng_proto::tractor::v1::TractorState;
using farm_ng_proto::tractor::v1::ViewDirection;
using farm_ng_proto::tractor::v1::ViewDirection_Name;
using farm_ng_proto::tractor::v1::ViewDirection_Parse;

DEFINE_string(video_dataset_result, "",
              "The path to a serialized CaptureVideoDatasetResult");
DEFINE_string(base_to_camera_result, "",
              "The path to a serialized CalibrateBaseToCameraResult");

DEFINE_bool(zero_indexed, true,
            "Data recorded with zero indexed video frame numbers?  This is a "
            "hack that should be removed once data format is stable.");
namespace farm_ng {

class ImageLoader {
 public:
  void OpenVideo(const Resource& resource) {
    CHECK_EQ(resource.content_type(), "video/mp4");

    if (!capture_) {
      video_name_ = (GetBlobstoreRoot() / resource.path()).string();
      capture_.reset(new cv::VideoCapture(video_name_));
    } else {
      if (video_name_ != (GetBlobstoreRoot() / resource.path()).string()) {
        capture_.reset(nullptr);
        OpenVideo(resource);
      }
    }
    CHECK(capture_->isOpened()) << "Video is not opened: " << video_name_;
  }

  cv::Mat LoadImage(const Image& image) {
    cv::Mat frame;
    if (image.resource().content_type() == "video/mp4") {
      OpenVideo(image.resource());
      VLOG(2) << image.resource().path()
              << " frame number: " << image.frame_number().value();

      int frame_number = image.frame_number().value();
      if (!FLAGS_zero_indexed) {
        CHECK_GT(frame_number, 0);
        frame_number -= 1;
      }
      capture_->set(cv::CAP_PROP_POS_FRAMES, frame_number);
      CHECK_EQ(frame_number, uint32_t(capture_->get(cv::CAP_PROP_POS_FRAMES)));
      *capture_ >> frame;
    } else {
      frame =
          cv::imread((GetBlobstoreRoot() / image.resource().path()).string(),
                     cv::IMREAD_UNCHANGED);
    }
    CHECK(!frame.empty());
    CHECK_EQ(frame.size().width, image.camera_model().image_width());
    CHECK_EQ(frame.size().height, image.camera_model().image_height());
    return frame;
  }
  std::unique_ptr<cv::VideoCapture> capture_;
  std::string video_name_;
};

struct FlowPointImage {
  uint64_t flow_point_id;
  Eigen::Vector2f image_coordinates;
};

class FlowPointWorld {
 public:
  uint64_t id;

  Eigen::Vector3d point_world;
  // set because we want it ordered.
  std::set<uint64_t> image_ids;
  double rmse = 0;
};

class FlowImage {
 public:
  uint64_t id;
  google::protobuf::Timestamp stamp;
  Sophus::SE3d camera_pose_world;
  std::optional<cv::Mat> image;
  cv::Mat debug_trails;
  std::unordered_map<uint64_t, FlowPointImage> flow_points;
};

template <typename Scalar>
inline cv::Point2f EigenToCvPoint2f(const Eigen::Matrix<Scalar, 2, 1>& x) {
  return cv::Point2f(x.x(), x.y());
}

template <typename Scalar>
inline cv::Point EigenToCvPoint(const Eigen::Matrix<Scalar, 2, 1>& x) {
  return cv::Point(int(x.x() + 0.5), int(x.y() + 0.5));
}

struct ProjectionCostFunctor {
  ProjectionCostFunctor(const CameraModel& camera_model,
                        const Eigen::Vector2d& point_image)
      : camera_model_(camera_model), point_image_(point_image) {}

  template <class T>
  bool operator()(T const* const raw_camera_pose_world,
                  T const* const raw_point_world, T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const camera_pose_world(
        raw_camera_pose_world);

    Eigen::Map<Eigen::Matrix<T, 3, 1> const> const point_world(raw_point_world);
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(raw_residuals);
    residuals =
        ProjectPointToPixel<T>(camera_model_, camera_pose_world * point_world) -
        point_image_.cast<T>();
    return true;
  }
  const CameraModel& camera_model_;
  Eigen::Vector2d point_image_;
};

struct PoseCostFunctor {
  PoseCostFunctor(const Sophus::SE3d& camera_start_pose_camera_end)
      : camera_end_pose_camera_start_(camera_start_pose_camera_end.inverse()) {}

  template <class T>
  bool operator()(T const* const raw_camera_pose_world_start,
                  T const* const raw_camera_pose_world_end,
                  T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const camera_pose_world_start(
        raw_camera_pose_world_start);

    Eigen::Map<Sophus::SE3<T> const> const camera_pose_world_end(
        raw_camera_pose_world_end);

    auto camera_start_pose_camera_end_est =
        camera_pose_world_start * camera_pose_world_end.inverse();

    auto camera_end_pose_camera_end_est =
        camera_end_pose_camera_start_.cast<T>() *
        camera_start_pose_camera_end_est;

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);
    residuals = T(100) * camera_end_pose_camera_end_est.log();
    return true;
  }
  Sophus::SE3d camera_end_pose_camera_start_;
};

typedef TimeSeries<BaseToCameraModel::WheelMeasurement>::RangeT
    WheelMeasurementRange;

struct FlowBlock {
  FlowImage* flow_image;
  FlowPointWorld* flow_point_world;
  FlowPointImage flow_point_image;
};

typedef std::unordered_map<uint64_t, std::vector<FlowBlock>> FlowBlocks;

class FlowBookKeeper {
 public:
  FlowBookKeeper(CameraModel camera_model) : camera_model_(camera_model) {
    cv::RNG rng;
    for (int i = 0; i < 1000; ++i) {
      colors_.push_back(cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                   rng.uniform(0, 255)));
    }
    int image_width = camera_model_.image_width();
    int image_height = camera_model_.image_height();

    lens_exclusion_mask_ = cv::imread(
        (GetBlobstoreRoot() / "configurations/tracking_mask.png").string(),
        cv::IMREAD_GRAYSCALE);
    if (lens_exclusion_mask_.empty()) {
      lens_exclusion_mask_ =
          cv::Mat::zeros(cv::Size(image_width, image_height), CV_8UC1);
      cv::circle(lens_exclusion_mask_,
                 cv::Point(image_width / 2, image_height / 2),
                 (image_width / 2.0) * 0.8, cv::Scalar::all(255), -1);
    }
  }

  // Add an image, assumed to be from the same camera, and close to periodic,
  // meaning from the same consecutive time series of images captured from the
  // camera.
  uint64_t AddImage(cv::Mat image, google::protobuf::Timestamp stamp,
                    Sophus::SE3d world_pose_camera) {
    CHECK_EQ(camera_model_.image_width(), image.size().width);
    CHECK_EQ(camera_model_.image_height(), image.size().height);
    CHECK_EQ(1, image.channels()) << "Expecting gray image.";
    FlowImage flow_image;
    flow_image.camera_pose_world = world_pose_camera.inverse();
    flow_image.image = image;
    flow_image.stamp = stamp;
    flow_image.id = image_id_gen_++;
    if (flow_image.id > 0) {
      FlowFromPrevious(&flow_image, false);
      flow_images_.at(flow_image.id - 1).image.reset();
      flow_images_.at(flow_image.id - 1).debug_trails = cv::Mat();
    }
    DetectGoodCorners(&flow_image);
    flow_images_.insert(std::make_pair(flow_image.id, flow_image));
    return flow_image.id;
  }

  std::pair<std::vector<cv::Point2f>, std::vector<uint64_t>> GetFlowCvPoints(
      const FlowImage& flow_image) const {
    std::vector<cv::Point2f> points;
    std::vector<uint64_t> ids;
    points.reserve(flow_image.flow_points.size());
    for (const auto& sample : flow_image.flow_points) {
      points.push_back(EigenToCvPoint2f(sample.second.image_coordinates));
      ids.push_back(sample.first);
    }
    return std::make_pair(points, ids);
  }

  void FlowFromPrevious(FlowImage* flow_image, bool debug = false) {
    CHECK_GT(flow_image->id, 0);
    CHECK_GT(flow_images_.count(flow_image->id - 1), 0);
    const FlowImage& prev = flow_images_.at(flow_image->id - 1);
    if (prev.flow_points.empty()) {
      return;
    }
    if (debug) {
      flow_image->debug_trails = prev.debug_trails;
    }
    CHECK(prev.image.has_value());
    std::vector<uchar> status;
    std::vector<float> err;
    auto prev_points_ids = GetFlowCvPoints(prev);
    const std::vector<cv::Point2f>& prev_points = prev_points_ids.first;
    std::vector<cv::Point2f> curr_points;
    std::vector<cv::Point2f> prev_bwd_points;

    cv::calcOpticalFlowPyrLK(*prev.image, *flow_image->image, prev_points,
                             curr_points, status, err);

    std::vector<uchar> bwd_status;

    cv::calcOpticalFlowPyrLK(*flow_image->image, *prev.image, curr_points,
                             prev_bwd_points, bwd_status, err);

    CHECK_EQ(curr_points.size(), prev_points.size());
    CHECK_EQ(prev.flow_points.size(), prev_points.size());
    CHECK_EQ(status.size(), prev_points.size());
    CHECK_EQ(err.size(), prev_points.size());

    cv::Mat debug_image;
    if (debug) {
      cv::cvtColor(*flow_image->image, debug_image, cv::COLOR_GRAY2BGR);
      if (flow_image->debug_trails.empty()) {
        flow_image->debug_trails =
            cv::Mat::zeros(flow_image->image->size(), CV_8UC3);
      }
      flow_image->debug_trails = flow_image->debug_trails * 0.9;
    }
    std::vector<cv::Point2f> prev_match, curr_match;
    std::vector<FlowPointImage> curr_flow_points;

    cv::Mat crowding_mask = cv::Mat::zeros(
        cv::Size(camera_model_.image_width(), camera_model_.image_height()),
        CV_8UC1);

    for (size_t i = 0; i < curr_points.size(); ++i) {
      if (!status[i] || !bwd_status[i]) {
        continue;
      }
      if (cv::norm(prev_bwd_points[i] - prev_points[i]) > 1.0) {
        continue;
      }
      CHECK(cv::Rect(cv::Point(0, 0), lens_exclusion_mask_.size())
                .contains(curr_points[i]));

      if (!lens_exclusion_mask_.at<uint8_t>(curr_points[i])) {
        continue;
      }

      // here we check if the curr_point[i] is not crowded by other flow points,
      // greedily.
      if (crowding_mask.at<uint8_t>(curr_points[i])) {
        continue;
      }
      // Mark a 20x20 pixel region as occupied in the crowding mask so that no
      // other points near this one may be added.
      // Crowding tends to occur when moving backwards,negatively along the
      // camera Z axis, as points that were close the camera get farther away
      // and closer together.
      int crowd_window = 41;
      cv::rectangle(crowding_mask,
                    cv::Rect(curr_points[i].x - crowd_window / 2,
                             curr_points[i].y - crowd_window / 2, crowd_window,
                             crowd_window),
                    cv::Scalar::all(255), -1);
      prev_match.push_back(prev_points[i]);
      curr_match.push_back(curr_points[i]);

      FlowPointImage flow_point =
          prev.flow_points.at(prev_points_ids.second[i]);
      flow_point.image_coordinates =
          Eigen::Vector2f(curr_points[i].x, curr_points[i].y);
      curr_flow_points.push_back(flow_point);
    }

    for (size_t i = 0; i < curr_flow_points.size(); ++i) {
      const auto& flow_point = curr_flow_points[i];

      flow_image->flow_points[flow_point.flow_point_id] = flow_point;
      auto world_it = flow_points_world_.find(flow_point.flow_point_id);
      CHECK(world_it != flow_points_world_.end());

      auto it_inserted = world_it->second.image_ids.insert(flow_image->id);
      CHECK(it_inserted.second);
      if (debug) {
        if (world_it->second.image_ids.size() >= 5) {
          cv::Scalar color = Color(flow_point.flow_point_id);
          cv::line(flow_image->debug_trails, prev_match[i], curr_match[i],
                   color);
          cv::circle(debug_image, curr_match[i], 3, color, -1);
        }
      }
    }

    if (debug) {
      debug_image = debug_image + flow_image->debug_trails;
      cv::imshow("debug flow", debug_image);
    }
  }

  void RemoveBlock(const FlowBlock& block) {
    block.flow_point_world->image_ids.erase(block.flow_image->id);
    block.flow_image->flow_points.erase(block.flow_point_world->id);
  }

  void RemovePointWorld(uint64_t id) {
    auto it = flow_points_world_.find(id);
    CHECK(it != flow_points_world_.end());
    for (uint64_t image_id : it->second.image_ids) {
      MutableFlowImage(image_id)->flow_points.erase(id);
    }
    flow_points_world_.erase(it);
  }

  void RenderMaskOfFlowPoints(const FlowImage& flow_image, cv::Mat* mask,
                              int window_size) {
    if (mask->empty()) {
      *mask = cv::Mat::ones(cv::Size(camera_model_.image_width(),
                                     camera_model_.image_height()),
                            CV_8UC1) *
              255;
    }
    for (const auto& flow_point : flow_image.flow_points) {
      cv::Point tl = EigenToCvPoint(flow_point.second.image_coordinates) -
                     cv::Point(window_size / 2, window_size / 2);
      cv::rectangle(*mask, cv::Rect(tl.x, tl.y, window_size, window_size),
                    cv::Scalar::all(0), -1);
    }
  }
  FlowPointImage GenFlowPoint(FlowImage* flow_image, cv::Point2f x) {
    FlowPointImage flow_point_image;
    flow_point_image.flow_point_id = flow_id_gen_++;
    flow_point_image.image_coordinates = Eigen::Vector2f(x.x, x.y);
    FlowPointWorld flow_point_world;
    flow_point_world.image_ids.insert(flow_image->id);
    flow_point_world.id = flow_point_image.flow_point_id;
    flow_point_world.point_world =
        flow_image->camera_pose_world.inverse() *
        ReprojectPixelToPoint<double>(
            camera_model_, flow_point_image.image_coordinates.cast<double>(),
            1.0);
    flow_points_world_.insert(
        std::make_pair(flow_point_image.flow_point_id, flow_point_world));
    flow_image->flow_points[flow_point_image.flow_point_id] = flow_point_image;
    return flow_point_image;
  }

  void DetectGoodCorners(FlowImage* flow_image) {
    CHECK(flow_image->image.has_value()) << "image must be set.";
    /// Parameters for Shi-Tomasi algorithm
    int max_corners = 400;

    double quality_level = 0.01;
    double min_distance = 10;
    int block_size = 3, gradient_size = 3;
    bool use_harris_detector = false;
    double k = 0.04;

    cv::Mat mask = lens_exclusion_mask_.clone();
    RenderMaskOfFlowPoints(*flow_image, &mask, 80);

    // cv::imshow("mask", mask);
    /// Apply corner detection
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(*flow_image->image, points, max_corners,
                            quality_level, min_distance, mask, block_size,
                            gradient_size, use_harris_detector, k);

    if (points.empty()) {
      return;
    }

    if (false) {
      cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                20, 0.03);
      cv::Size subPixWinSize(10, 10);

      cv::cornerSubPix(*flow_image->image, points, subPixWinSize,
                       cv::Size(-1, -1), termcrit);
    }
    for (auto& point : points) {
      GenFlowPoint(flow_image, point);
    }
  }

  uint64_t LastImageId() const {
    CHECK_GT(image_id_gen_, 0);
    return image_id_gen_ - 1;
  }

  const FlowImage* PreviousFlowImage() const {
    if (flow_images_.empty()) {
      return nullptr;
    }
    CHECK(image_id_gen_ > 0);
    auto it = flow_images_.find(image_id_gen_ - 1);
    CHECK(it != flow_images_.end()) << image_id_gen_;
    return &it->second;
  }

  FlowImage* MutablePreviousFlowImage() {
    if (flow_images_.empty()) {
      return nullptr;
    }
    CHECK(image_id_gen_ > 0);
    auto it = flow_images_.find(image_id_gen_ - 1);
    CHECK(it != flow_images_.end()) << image_id_gen_;
    return &it->second;
  }

  FlowImage* MutableFlowImage(uint64_t id) {
    CHECK(!flow_images_.empty());
    CHECK(image_id_gen_ > 0);
    CHECK(id < image_id_gen_);
    auto it = flow_images_.find(id);
    CHECK(it != flow_images_.end()) << id;
    return &it->second;
  }

  FlowPointWorld* MutableFlowPointWorld(uint64_t id) {
    CHECK(!flow_points_world_.empty());
    CHECK(flow_id_gen_ > 0);
    auto it = flow_points_world_.find(id);
    CHECK(it != flow_points_world_.end());
    return &it->second;
  }
  const std::unordered_map<uint64_t, FlowPointWorld>& FlowPointsWorld() const {
    return flow_points_world_;
  }

  cv::Scalar Color(uint64_t id) const { return colors_[id % colors_.size()]; }

 private:
  CameraModel camera_model_;
  cv::Mat lens_exclusion_mask_;
  std::vector<cv::Scalar> colors_;
  uint64_t image_id_gen_ = 0;
  uint64_t flow_id_gen_ = 0;
  std::unordered_map<uint64_t, FlowPointWorld> flow_points_world_;
  std::unordered_map<uint64_t, FlowImage> flow_images_;
};

void SavePly(std::string ply_path, const std::vector<Eigen::Vector3d>& points) {
  LOG(INFO) << ply_path << " npoints: " << points.size();
  std::ofstream out(ply_path);
  out << "ply\n";
  out << "format ascii 1.0\n";
  out << "element vertex " << points.size() << "\n";
  out << "property float x\n";
  out << "property float y\n";
  out << "property float z\n";
  out << "end_header\n";
  for (auto p : points) {
    out << float(p.x()) << " " << float(p.y()) << " " << float(p.z()) << "\n";
  }
  out.close();
}

class VisualOdometer {
 public:
  VisualOdometer(const CameraModel& camera_model,
                 const BaseToCameraModel& base_to_camera_model)
      : camera_model_(camera_model),
        flow_(camera_model),
        base_to_camera_model_(base_to_camera_model),
        odometry_pose_base_(Sophus::SE3d::rotX(0.0)) {
    ProtoToSophus(base_to_camera_model_.base_pose_camera().a_pose_b(),
                  &base_pose_camera_);
  }

  void AddWheelMeasurements(
      const BaseToCameraModel::WheelMeasurement& measurements) {
    wheel_measurements_.insert(measurements);
  }

  void AddImage(cv::Mat image, google::protobuf::Timestamp stamp) {
    if (const FlowImage* prev_flow_image = flow_.PreviousFlowImage()) {
      auto wheel_measurements =
          wheel_measurements_.find_range(prev_flow_image->stamp, stamp);

      Sophus::SE3d base_pose_basep = TractorStartPoseTractorEnd(
          base_to_camera_model_.wheel_radius(),
          base_to_camera_model_.wheel_baseline(), wheel_measurements.first,
          wheel_measurements.second);
      Sophus::SE3d odometry_pose_base_wheel_only =
          (base_pose_camera_ * prev_flow_image->camera_pose_world).inverse() *
          base_pose_basep;

      odometry_pose_base_ = odometry_pose_base_wheel_only;
      if (base_pose_basep.log().norm() > 0.001) {
        flow_.AddImage(image, stamp,
                       odometry_pose_base_wheel_only * base_pose_camera_);
        SolvePose();
        if (flow_.LastImageId() % 100 == 0) {
          DumpFlowPointsWorld("/tmp/flow_points_world." +
                              std::to_string(flow_.LastImageId()) + ".ply");
        }
      }

    } else {
      flow_.AddImage(image, stamp, odometry_pose_base_ * base_pose_camera_);
    }
  }

  void AddFlowBlockToProblem(ceres::Problem* problem,
                             const FlowBlock& flow_block) {
    ceres::CostFunction* cost_function1 =
        new ceres::AutoDiffCostFunction<ProjectionCostFunctor, 2,
                                        Sophus::SE3d::num_parameters, 3>(
            new ProjectionCostFunctor(
                camera_model_,
                flow_block.flow_point_image.image_coordinates.cast<double>()));

    problem->AddParameterBlock(flow_block.flow_point_world->point_world.data(),
                               3);
    problem->AddResidualBlock(cost_function1, new ceres::CauchyLoss(2),
                              flow_block.flow_image->camera_pose_world.data(),
                              flow_block.flow_point_world->point_world.data());
  }

  void AddFlowImageToProblem(FlowImage* flow_image, ceres::Problem* problem,
                             FlowBlocks* flow_blocks) {
    problem->AddParameterBlock(flow_image->camera_pose_world.data(),
                               Sophus::SE3d::num_parameters,
                               new LocalParameterizationSE3);

    for (const auto& id_flow_point : flow_image->flow_points) {
      const FlowPointImage& flow_point = id_flow_point.second;

      FlowPointWorld* flow_point_world =
          flow_.MutableFlowPointWorld(flow_point.flow_point_id);
      if (flow_point_world->image_ids.size() < 5) {
        continue;
      }
      if (flow_blocks->size() < 250 ||
          flow_blocks->count(flow_point_world->id)) {
        FlowBlock flow_block({flow_image, flow_point_world, flow_point});
        (*flow_blocks)[flow_point_world->id].push_back(flow_block);
        AddFlowBlockToProblem(problem, flow_block);
      }
    }
  }
  void SolvePose(bool debug = true) {
    ceres::Problem problem;

    std::set<uint64_t> flow_image_ids;
    FlowBlocks flow_blocks;
    {
      uint64_t image_id(flow_.LastImageId());
      if (image_id < 5) {
        return;
      }
      uint64_t begin_id = std::max(0, int(image_id) - 50);
      flow_image_ids.insert(image_id);
      if (goal_image_id_) {
        if (int(image_id) - int(*goal_image_id_) < 50) {
          flow_image_ids.insert(*goal_image_id_);
        }
      }
      uint64_t skip = std::max(1, (int(image_id) - int(begin_id)) / 5);
      while (begin_id < image_id) {
        flow_image_ids.insert(begin_id);
        begin_id += skip;
      }
    }
    for (auto image_id : flow_image_ids) {
      FlowImage* flow_image = flow_.MutableFlowImage(image_id);
      AddFlowImageToProblem(flow_image, &problem, &flow_blocks);
    }

    // debugging which images are used.
    if (false) {
      std::stringstream ss;
      for (auto id : flow_image_ids) {
        ss << " " << id;
      }
      LOG(INFO) << "Num images: " << flow_image_ids.size() << ss.str();
    }

    for (auto start = flow_image_ids.begin(), end = ++flow_image_ids.begin();
         end != flow_image_ids.end(); ++start, ++end) {
      FlowImage* flow_image_start = flow_.MutableFlowImage(*start);
      FlowImage* flow_image_end = flow_.MutableFlowImage(*end);

      auto wheel_measurements = wheel_measurements_.find_range(
          flow_image_start->stamp, flow_image_end->stamp);

      Sophus::SE3d base_start_pose_base_end = TractorStartPoseTractorEnd(
          base_to_camera_model_.wheel_radius(),
          base_to_camera_model_.wheel_baseline(), wheel_measurements.first,
          wheel_measurements.second);

      Sophus::SE3d camera_start_pose_camera_end = base_pose_camera_.inverse() *
                                                  base_start_pose_base_end *
                                                  base_pose_camera_;
      ceres::CostFunction* cost_function1 =
          new ceres::AutoDiffCostFunction<PoseCostFunctor, 6,
                                          Sophus::SE3d::num_parameters,
                                          Sophus::SE3d::num_parameters>(
              new PoseCostFunctor(camera_start_pose_camera_end));

      problem.AddResidualBlock(cost_function1, new ceres::CauchyLoss(1.0),
                               flow_image_start->camera_pose_world.data(),
                               flow_image_end->camera_pose_world.data());
    }

    if (false && goal_image_id_ && flow_image_ids.count(*goal_image_id_)) {
      problem.SetParameterBlockConstant(
          flow_.MutableFlowImage(*goal_image_id_)->camera_pose_world.data());
    }

    // Set solver options (precision / method)
    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_SCHUR;
    options.gradient_tolerance = 1e-4;
    options.function_tolerance = 1e-4;
    options.parameter_tolerance = 1e-4;
    options.max_num_iterations = 200;

    // Solve
    ceres::Solver::Summary summary;
    // options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.minimizer_progress_to_stdout = false;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
      LOG(INFO) << summary.FullReport();
    }

    double all_rmse = 0;
    double all_n = 0;
    for (auto id_blocks : flow_blocks) {
      uint64_t point_world_id = id_blocks.first;
      double rmse = 0;
      double n = 0;
      for (auto block : id_blocks.second) {
        Eigen::Vector3d point_camera = block.flow_image->camera_pose_world *
                                       block.flow_point_world->point_world;
        Eigen::Vector2d point_image_proj =
            ProjectPointToPixel(camera_model_, point_camera);

        if (point_camera.z() < 0) {
          flow_.RemoveBlock(block);
          continue;
        }

        if (point_camera.norm() > 5e2) {
          flow_.RemoveBlock(block);
          continue;
        }
        double err2 = (point_image_proj -
                       block.flow_point_image.image_coordinates.cast<double>())
                          .squaredNorm();
        if (err2 > (4 * 4)) {
          flow_.RemoveBlock(block);
        } else {
          rmse += err2;
          n += 1;
        }
      }
      auto flow_point_world = flow_.MutableFlowPointWorld(point_world_id);
      if (flow_point_world->image_ids.size() < 5 ||
          flow_point_world->point_world.norm() > 1e3) {
        flow_.RemovePointWorld(point_world_id);
      } else {
        flow_.MutableFlowPointWorld(point_world_id)->rmse = std::sqrt(rmse / n);
        all_rmse += rmse;
        all_n += n;
      }
    }
    VLOG(2) << "RMSE: " << std::sqrt(all_rmse / std::max(all_n, 1.0))
            << " N: " << all_n;

    FlowImage* flow_image = flow_.MutablePreviousFlowImage();

    if (goal_image_id_) {
      auto base_pose_world_goal =
          camera_pose_base_goal_.inverse() *
          flow_.MutableFlowImage(*goal_image_id_)->camera_pose_world;

      auto camera_pose_base_goal =
          flow_image->camera_pose_world * base_pose_world_goal.inverse();

      goal_image_id_ = flow_image->id;
      camera_pose_base_goal_ = camera_pose_base_goal;
    }

    if (debug) {
      cv::Mat reprojection_image;

      cv::cvtColor(*(flow_image->image), reprojection_image,
                   cv::COLOR_GRAY2BGR);

      for (const auto& id_flow_point : flow_image->flow_points) {
        const auto& flow_point = id_flow_point.second;
        FlowPointWorld* flow_point_world =
            flow_.MutableFlowPointWorld(flow_point.flow_point_id);
        if (flow_point_world->rmse == 0.0) {
          continue;
        }
        Eigen::Vector2d point_image_proj = ProjectPointToPixel(
            camera_model_,
            flow_image->camera_pose_world * flow_point_world->point_world);
        cv::line(reprojection_image,
                 EigenToCvPoint(flow_point.image_coordinates),
                 EigenToCvPoint(point_image_proj),
                 flow_.Color(flow_point.flow_point_id));

        cv::circle(reprojection_image, EigenToCvPoint(point_image_proj), 2,
                   flow_.Color(flow_point.flow_point_id), -1);
        cv::circle(reprojection_image,
                   EigenToCvPoint(flow_point.image_coordinates), 5,
                   flow_.Color(flow_point.flow_point_id));
      }
      if (goal_image_id_) {
        for (int i = 0; i < 1000; ++i) {
          Eigen::Vector3d p1 =
              camera_pose_base_goal_ * Eigen::Vector3d(0.1 * i, 0.0, 0.0);
          Eigen::Vector3d p2 =
              camera_pose_base_goal_ * Eigen::Vector3d(0.1 * (i + 1), 0.0, 0.0);
          if (p1.z() > 0.0 && p2.z() > 0.00) {
            cv::line(reprojection_image,
                     EigenToCvPoint(ProjectPointToPixel(camera_model_, p1)),
                     EigenToCvPoint(ProjectPointToPixel(camera_model_, p2)),
                     cv::Scalar(0, 255, 0), 3);
          }
        }
      }
      auto camera_pose_base = base_pose_camera_.inverse();
      for (int i = 0; i < 1000; ++i) {
        Eigen::Vector3d ap1 =
            camera_pose_base * Eigen::Vector3d(0.1 * i, 0.0, 0.0);
        Eigen::Vector3d ap2 =
            camera_pose_base * Eigen::Vector3d(0.1 * (i + 1), 0.0, 0.0);
        cv::line(reprojection_image,
                 EigenToCvPoint(ProjectPointToPixel(camera_model_, ap1)),
                 EigenToCvPoint(ProjectPointToPixel(camera_model_, ap2)),
                 cv::Scalar(0, 0, 255), 1);
      }

      cv::flip(reprojection_image, reprojection_image, -1);
      if (!writer) {
        writer.reset(new cv::VideoWriter(
            video_writer_dev,
            0,   // fourcc
            30,  // fps
            cv::Size(camera_model_.image_width(), camera_model_.image_height()),
            true));
      }
      writer->write(reprojection_image);
      cv::imshow("reprojection", reprojection_image);
    }
  }

  void DumpFlowPointsWorld(std::string ply_path) {
    std::vector<Eigen::Vector3d> points;
    for (auto it : flow_.FlowPointsWorld()) {
      if (it.second.image_ids.size() >= 5) {
        points.push_back(it.second.point_world);
      }
    }
    SavePly(ply_path, points);
  }

  void SetGoal() {
    if (flow_.LastImageId() < 10) {
      return;
    }
    goal_image_id_ = flow_.LastImageId();
    camera_pose_base_goal_ = base_pose_camera_.inverse();
  }

 private:
  CameraModel camera_model_;
  FlowBookKeeper flow_;
  BaseToCameraModel base_to_camera_model_;

  Sophus::SE3d base_pose_camera_;
  Sophus::SE3d odometry_pose_base_;

  std::optional<uint64_t> goal_image_id_;
  Sophus::SE3d camera_pose_base_goal_;
  TimeSeries<BaseToCameraModel::WheelMeasurement> wheel_measurements_;

  std::string video_writer_dev =
      std::string("appsrc !") + " videoconvert ! x264enc ! " +
      " mp4mux ! filesink location=" + "/tmp/out.mp4";

  std::unique_ptr<cv::VideoWriter> writer;
};

class TrackingStudyProgram {
 public:
  TrackingStudyProgram(EventBus& bus)
      : bus_(bus), timer_(bus_.get_io_service()) {
    on_timer(boost::system::error_code());
  }

  int run() {
    auto dataset_result = ReadProtobufFromJsonFile<CaptureVideoDatasetResult>(
        FLAGS_video_dataset_result);

    auto base_to_camera_result =
        ReadProtobufFromJsonFile<CalibrateBaseToCameraResult>(
            FLAGS_base_to_camera_result);

    auto base_to_camera_model = ReadProtobufFromResource<BaseToCameraModel>(
        base_to_camera_result.base_to_camera_model_solved());

    EventLogReader log_reader(dataset_result.dataset());

    std::unique_ptr<VisualOdometer> vo;
    ImageLoader image_loader;

    int skip_frame_counter = 0;
    while (true) {
      try {
        auto event = log_reader.ReadNext();
        TractorState state;
        if (event.data().UnpackTo(&state) && vo) {
          BaseToCameraModel::WheelMeasurement wheel_measurement;
          CopyTractorStateToWheelState(state, &wheel_measurement);
          vo->AddWheelMeasurements(wheel_measurement);
        }
        Image image;
        if (event.data().UnpackTo(&image) && skip_frame_counter++ % 1 == 0) {
          if (!vo) {
            vo.reset(
                new VisualOdometer(image.camera_model(), base_to_camera_model));
          }
          cv::Mat gray = image_loader.LoadImage(image);

          cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

          google::protobuf::Timestamp last_stamp = MakeTimestampNow();
          vo->AddImage(gray, event.stamp());
          auto now = MakeTimestampNow();
          LOG(INFO) << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                           now - last_stamp)
                    << " ms";

          int key = cv::waitKey(1) & 0xff;
          if (key == ' ') {
            vo->SetGoal();
          }
          if (key == 'q') {
            break;
          }
        }
        bus_.get_io_service().poll();

      } catch (std::runtime_error& e) {
        LOG(INFO) << e.what();
        bus_.get_io_service().stop();
        bus_.get_io_service().reset();
        break;
      }
    }

    vo->DumpFlowPointsWorld("/tmp/flow_points_world.final.ply");
    return 0;
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&TrackingStudyProgram::on_timer, this,
                                std::placeholders::_1));
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
};  // namespace farm_ng

}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) { LOG(INFO) << "Cleanup."; }

int Main(farm_ng::EventBus& bus) {
  farm_ng::TrackingStudyProgram program(bus);
  return program.run();
}
int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
