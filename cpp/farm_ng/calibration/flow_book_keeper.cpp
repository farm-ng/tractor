#include "farm_ng/calibration/flow_book_keeper.h"

#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include "farm_ng/blobstore.h"

#include "farm_ng/calibration/camera_model.h"
#include "farm_ng/calibration/eigen_cv.h"

namespace farm_ng {

FlowBookKeeper::FlowBookKeeper(CameraModel camera_model, size_t max_history)
    : max_history_(max_history),
      camera_model_(camera_model),
      flow_window_(21, 21),
      flow_max_levels_(3) {
  cv::RNG rng;
  for (int i = 0; i < 1000; ++i) {
    colors_.push_back(cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                 rng.uniform(0, 255)));
  }
  int image_width = camera_model_.image_width();
  int image_height = camera_model_.image_height();
  std::string mask_path =
      (GetBlobstoreRoot() / "configurations/tracking_mask.png").string();
  lens_exclusion_mask_ = cv::imread(mask_path,

                                    cv::IMREAD_GRAYSCALE);
  if (lens_exclusion_mask_.empty()) {
    lens_exclusion_mask_ =
        cv::Mat::zeros(cv::Size(image_width, image_height), CV_8UC1);
    cv::circle(lens_exclusion_mask_,
               cv::Point(image_width / 2, image_height / 2),
               (image_width / 2.0) * 0.8, cv::Scalar::all(255), -1);
    cv::imwrite(mask_path, lens_exclusion_mask_);
  }
}

// Add an image, assumed to be from the same camera, and close to periodic,
// meaning from the same consecutive time series of images captured from the
// camera.
uint64_t FlowBookKeeper::AddImage(cv::Mat image,
                                  google::protobuf::Timestamp stamp,
                                  Sophus::SE3d world_pose_camera, bool debug) {
  CHECK_EQ(camera_model_.image_width(), image.size().width);
  CHECK_EQ(camera_model_.image_height(), image.size().height);
  CHECK_EQ(1, image.channels()) << "Expecting gray image.";
  FlowImage flow_image;
  flow_image.camera_pose_world = world_pose_camera.inverse();
  flow_image.image = image;
  std::vector<cv::Mat> pyramid;
  cv::buildOpticalFlowPyramid(image, pyramid, flow_window_, flow_max_levels_);
  flow_image.pyramid = pyramid;

  flow_image.stamp = stamp;
  flow_image.id = image_id_gen_++;
  if (flow_image.id > 0) {
    FlowFromPrevious(&flow_image, debug);
    flow_images_.at(flow_image.id - 1).image.reset();
    flow_images_.at(flow_image.id - 1).pyramid.reset();
    flow_images_.at(flow_image.id - 1).debug_trails = cv::Mat();
  }
  DetectGoodCorners(&flow_image);
  flow_images_.insert(std::make_pair(flow_image.id, flow_image));
  flow_image_ids_.insert(flow_image.id);
  while (flow_images_.size() > max_history_) {
    uint64_t earliest_image_id = *flow_image_ids_.begin();
    RemoveFlowImage(earliest_image_id);
  }
  return flow_image.id;
}

void FlowBookKeeper::RemoveFlowImage(uint64_t id) {
  auto id_it = flow_image_ids_.find(id);
  CHECK(id_it != flow_image_ids_.end());
  flow_image_ids_.erase(id_it);
  auto image_it = flow_images_.find(id);
  CHECK(image_it != flow_images_.end());
  for (const auto& flow_pt : image_it->second.flow_points) {
    auto flow_point_world = flow_points_world_.find(flow_pt.first);
    CHECK(flow_point_world != flow_points_world_.end());
    auto fimage_id = flow_point_world->second.image_ids.find(id);
    CHECK(fimage_id != flow_point_world->second.image_ids.end());
    flow_point_world->second.image_ids.erase(fimage_id);
    if (flow_point_world->second.image_ids.empty()) {
      RemovePointWorld(flow_point_world->second.id);
    }
  }
  flow_images_.erase(image_it);
}

std::pair<std::vector<cv::Point2f>, std::vector<uint64_t>>
FlowBookKeeper::GetFlowCvPoints(const FlowImage& flow_image) const {
  std::vector<cv::Point2f> points;
  std::vector<uint64_t> ids;
  points.reserve(flow_image.flow_points.size());
  for (const auto& sample : flow_image.flow_points) {
    points.push_back(EigenToCvPoint2f(sample.second.image_coordinates));
    ids.push_back(sample.first);
  }
  return std::make_pair(points, ids);
}

void FlowBookKeeper::FlowFromPrevious(FlowImage* flow_image, bool debug) {
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

  cv::calcOpticalFlowPyrLK(*prev.pyramid, *flow_image->pyramid, prev_points,
                           curr_points, status, err);

  std::vector<uchar> bwd_status;

  cv::calcOpticalFlowPyrLK(*flow_image->pyramid, *prev.pyramid, curr_points,
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
    int crowd_window = 31;
    cv::rectangle(crowding_mask,
                  cv::Rect(curr_points[i].x - crowd_window / 2,
                           curr_points[i].y - crowd_window / 2, crowd_window,
                           crowd_window),
                  cv::Scalar::all(255), -1);
    prev_match.push_back(prev_points[i]);
    curr_match.push_back(curr_points[i]);

    FlowPointImage flow_point = prev.flow_points.at(prev_points_ids.second[i]);
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
        cv::line(flow_image->debug_trails, prev_match[i], curr_match[i], color);
        cv::circle(debug_image, curr_match[i], 3, color, -1);
      }
    }
  }

  if (debug) {
    debug_image_ = debug_image + flow_image->debug_trails;
  }
}

void FlowBookKeeper::RemoveBlock(const FlowBlock& block) {
  block.flow_point_world->image_ids.erase(block.flow_image->id);
  block.flow_image->flow_points.erase(block.flow_point_world->id);
}

void FlowBookKeeper::RemovePointWorld(uint64_t id) {
  auto it = flow_points_world_.find(id);
  CHECK(it != flow_points_world_.end());
  for (uint64_t image_id : it->second.image_ids) {
    MutableFlowImage(image_id)->flow_points.erase(id);
  }
  flow_points_world_.erase(it);
}

void FlowBookKeeper::RenderMaskOfFlowPoints(const FlowImage& flow_image,
                                            cv::Mat* mask, int window_size) {
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
FlowPointImage FlowBookKeeper::GenFlowPoint(FlowImage* flow_image,
                                            cv::Point2f x) {
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

void FlowBookKeeper::DetectGoodCorners(FlowImage* flow_image) {
  CHECK(flow_image->image.has_value()) << "image must be set.";
  /// Parameters for Shi-Tomasi algorithm
  int max_corners = 400;

  double quality_level = 0.005;
  double min_distance = 10;
  int block_size = 3, gradient_size = 3;
  bool use_harris_detector = false;
  double k = 0.04;
  bool non_max = true;
  int fast_threshold = 5;
  cv::Mat mask = lens_exclusion_mask_.clone();
  RenderMaskOfFlowPoints(*flow_image, &mask, 80);
  cv::Mat detect_image = *(flow_image->image) & mask;

  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(detect_image, keypoints, fast_threshold, non_max);
  LOG(INFO) << "keypoints.size() " << keypoints.size();


      size_t fast_count = 0;
  for (const auto& kpt : keypoints) {
   
    
    // here we check if the curr_point[i] is not crowded by other flow points,
    // greedily.
    cv::Point pt(kpt.pt.x + 0.5f, kpt.pt.y+0.5f);
    if(!cv::Rect(0,0, mask.size().width, mask.size().height).contains(pt)) {
      continue;

    }
    if (!mask.at<uint8_t>(pt)) {
      continue;
    }
  
++fast_count;
    // Mark a 20x20 pixel region as occupied in the crowding mask so that no
    // other points near this one may be added.
    // Crowding tends to occur when moving backwards,negatively along the
    // camera Z axis, as points that were close the camera get farther away
    // and closer together.
    int crowd_window = 11;
    cv::rectangle(mask,
                  cv::Rect(pt.x - crowd_window / 2,
                           pt.y - crowd_window / 2, crowd_window,
                           crowd_window),
                  cv::Scalar::all(0), -1);
    GenFlowPoint(flow_image, kpt.pt);
  }
    LOG(INFO) << "keypoints.size() after filter " << fast_count;

  return;
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
const FlowImage* FlowBookKeeper::EarliestFlowImage() const {
  uint64_t earliest_image_id = *flow_image_ids_.begin();
  return const_cast<FlowBookKeeper*>(this)->MutableFlowImage(earliest_image_id);
}

uint64_t FlowBookKeeper::LastImageId() const {
  CHECK_GT(image_id_gen_, 0);
  return image_id_gen_ - 1;
}

const FlowImage* FlowBookKeeper::PreviousFlowImage() const {
  if (flow_images_.empty()) {
    return nullptr;
  }
  CHECK(image_id_gen_ > 0);
  auto it = flow_images_.find(image_id_gen_ - 1);
  CHECK(it != flow_images_.end()) << image_id_gen_;
  return &it->second;
}

FlowImage* FlowBookKeeper::MutablePreviousFlowImage() {
  if (flow_images_.empty()) {
    return nullptr;
  }
  CHECK(image_id_gen_ > 0);
  auto it = flow_images_.find(image_id_gen_ - 1);
  CHECK(it != flow_images_.end()) << image_id_gen_;
  return &it->second;
}

FlowImage* FlowBookKeeper::MutableFlowImage(uint64_t id) {
  CHECK(!flow_images_.empty());
  CHECK(image_id_gen_ > 0);
  CHECK(id < image_id_gen_);
  auto it = flow_images_.find(id);
  CHECK(it != flow_images_.end()) << id;
  return &it->second;
}

FlowPointWorld* FlowBookKeeper::MutableFlowPointWorld(uint64_t id) {
  CHECK(!flow_points_world_.empty());
  CHECK(flow_id_gen_ > 0);
  auto it = flow_points_world_.find(id);
  CHECK(it != flow_points_world_.end());
  return &it->second;
}
const std::unordered_map<uint64_t, FlowPointWorld>&
FlowBookKeeper::FlowPointsWorld() const {
  return flow_points_world_;
}

cv::Scalar FlowBookKeeper::Color(uint64_t id) const {
  return colors_[id % colors_.size()];
}

}  // namespace farm_ng
