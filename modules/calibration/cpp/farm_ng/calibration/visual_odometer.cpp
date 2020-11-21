#include "farm_ng/calibration/visual_odometer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include <Eigen/Geometry>

#include <ceres/ceres.h>

#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/eigen_cv.h"
#include "farm_ng/perception/sophus_protobuf.h"

using farm_ng::core::MakeTimestampNow;
using farm_ng::perception::CameraModel;
using farm_ng::perception::EigenToCvPoint;
using farm_ng::perception::EigenToCvPoint2f;
using farm_ng::perception::ProjectPointToPixel;

namespace farm_ng {
namespace calibration {

namespace {

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

}  // namespace

VisualOdometer::VisualOdometer(const CameraModel& camera_model,
                               const BaseToCameraModel& base_to_camera_model,
                               size_t max_history)
    : camera_model_(camera_model),
      flow_(camera_model, max_history),
      base_to_camera_model_(base_to_camera_model),
      odometry_pose_base_(Sophus::SE3d::rotX(0.0)) {
  ProtoToSophus(base_to_camera_model_.base_pose_camera().a_pose_b(),
                &base_pose_camera_);
}

void VisualOdometer::AddWheelMeasurements(
    const BaseToCameraModel::WheelMeasurement& measurements) {
  wheel_measurements_.insert(measurements);
}

VisualOdometerResult VisualOdometer::AddImage(
    cv::Mat image, google::protobuf::Timestamp stamp) {
  VisualOdometerResult result;
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
    if ((true || base_pose_basep.log().norm() > 0.001) &&
        !wheel_measurements_.empty()) {
      auto start = MakeTimestampNow();

      flow_.AddImage(image, stamp,
                     odometry_pose_base_wheel_only * base_pose_camera_, true);
      debug_image_ = flow_.GetDebugImage();
      auto after_flow = MakeTimestampNow();

      SolvePose(true);
      odometry_pose_base_ =
          (base_pose_camera_ * flow_.PreviousFlowImage()->camera_pose_world)
              .inverse();

      wheel_measurements_.RemoveBefore(flow_.EarliestFlowImage()->stamp);
      auto after_solve = MakeTimestampNow();
      LOG_EVERY_N(INFO, 100)
          << "VO took: "
          << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                 after_solve - start)
          << " ms flow: "
          << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                 after_flow - start)
          << " ms solve: "
          << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                 after_solve - after_flow);

      if (false && flow_.LastImageId() % 100 == 0) {
        DumpFlowPointsWorld("/tmp/flow_points_world." +
                            std::to_string(flow_.LastImageId()) + ".ply");
      }
    }

  } else {
    flow_.AddImage(image, stamp, odometry_pose_base_ * base_pose_camera_, true);
    debug_image_ = flow_.GetDebugImage();
  }

  if (goal_image_id_) {
    // TODO(ethanrublee) this is goal pose stuff here is a HACK, should be moved
    // somewhere else, tracking camera can produce some generic poses which can
    // be used for a variety of control and path following modes.
    //
    // Here the base_pose_goal is the start of the planned path (parameterized
    // as the x+ axis of this frame). For convenience downstream, we'll project
    // the tractor's goal (nearest point on this path + some reasonable
    // distance) onto this path.
    Sophus::SE3d base_pose_goal = base_pose_camera_ * camera_pose_base_goal_;
    // The path is the parameterized line passing through the origin of goal
    // frame and along the X axis.
    // This line is in the base frame.
    auto path_base = Eigen::ParametrizedLine<double, 3>::Through(
        base_pose_goal.translation(),
        base_pose_goal * Eigen::Vector3d(1.0, 0, 0));

    // Project the origin of the base frame onto the line, this is the closest
    // point on the path. Here we transform the point into the goal's reference
    // frame. We expect this point to be non zero in X, and zero in y,z as it
    // lies on the X-axis of the goal reference frame.
    Eigen::Vector3d closest_path_point_goal =
        base_pose_goal.inverse() *
        path_base.projection(Eigen::Vector3d(0, 0, 0));
    CHECK_NEAR(closest_path_point_goal.y(), 0.0, 1e-6);
    CHECK_NEAR(closest_path_point_goal.z(), 0.0, 1e-6);

    // Here we add 3 meters to the nearest point on the line.  Think of this
    // like a carrot on a stick hanging 3 meters front of the robot on the
    // planned path line.
    // NOTE this distance will effect the move to goal controller behavior.  If
    // its large, then the heading corrections will be less and the tractor will
    // gradually correct to get onto the path.  If its too small, the tractor
    // will pivot dramatically trying to stay on the line.

    Sophus::SE3d base_pose_goal_carrot =
        base_pose_goal *
        Sophus::SE3d::transX(closest_path_point_goal.x() + 10.0);
    SophusToProto(base_pose_goal_carrot,
                  result.base_pose_goal.mutable_a_pose_b());
    if (!debug_image_.empty()) {
      cv::circle(debug_image_,
                 EigenToCvPoint(ProjectPointToPixel(
                     camera_model_, base_pose_camera_.inverse() *
                                        base_pose_goal_carrot.translation())),
                 10, cv::Scalar(255, 0, 0), 3);
    }
  } else {
    SophusToProto(Sophus::SE3d::rotZ(0.0),
                  result.base_pose_goal.mutable_a_pose_b());
  }
  result.base_pose_goal.mutable_a_pose_b()->mutable_stamp()->CopyFrom(stamp);
  result.base_pose_goal.set_frame_a("tractor/base");
  result.base_pose_goal.set_frame_b("goal");

  SophusToProto(odometry_pose_base_,
                result.odometry_vo_pose_base.mutable_a_pose_b());
  result.odometry_vo_pose_base.mutable_a_pose_b()->mutable_stamp()->CopyFrom(
      stamp);
  result.odometry_vo_pose_base.set_frame_a("odometry/vo");
  result.odometry_vo_pose_base.set_frame_b("tractor/base");

  return result;
}

void VisualOdometer::AddFlowBlockToProblem(ceres::Problem* problem,
                                           const FlowBlock& flow_block) {
  ceres::CostFunction* cost_function1 =
      new ceres::AutoDiffCostFunction<ProjectionCostFunctor, 2,
                                      Sophus::SE3d::num_parameters, 3>(
          new ProjectionCostFunctor(
              camera_model_,
              flow_block.flow_point_image.point_image.cast<double>()));

  problem->AddParameterBlock(flow_block.flow_point_world->point_world.data(),
                             3);
  problem->AddResidualBlock(cost_function1, new ceres::CauchyLoss(2),
                            flow_block.flow_image->camera_pose_world.data(),
                            flow_block.flow_point_world->point_world.data());
}

void VisualOdometer::AddFlowImageToProblem(FlowImage* flow_image,
                                           ceres::Problem* problem,
                                           FlowBlocks* flow_blocks) {
  problem->AddParameterBlock(flow_image->camera_pose_world.data(),
                             Sophus::SE3d::num_parameters,
                             new LocalParameterizationSE3);

  for (const auto& id_flow_point : flow_image->flow_points) {
    const FlowPointImage& flow_point = id_flow_point.second;

    FlowPointWorld* flow_point_world =
        flow_.MutableFlowPointWorld(flow_point.id);
    if (flow_point_world->image_ids.size() < 5) {
      continue;
    }
    if (true) {  // flow_blocks->size() < 100 ||
                 // flow_blocks->count(flow_point_world->id)) {
      FlowBlock flow_block({flow_image, flow_point_world, flow_point});
      (*flow_blocks)[flow_point_world->id].push_back(flow_block);
      AddFlowBlockToProblem(problem, flow_block);
    }
  }
}
void VisualOdometer::SolvePose(bool debug) {
  ceres::Problem problem;

  std::set<uint64_t> flow_image_ids;
  FlowBlocks flow_blocks;
  {
    uint64_t image_id(flow_.LastImageId());
    if (image_id < 5) {
      return;
    }
    uint64_t begin_id =
        std::max(int(flow_.EarliestFlowImage()->id), int(image_id) - 50);
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
  // debugging which images are used.
  if (false) {
    std::stringstream ss;
    for (auto id : flow_image_ids) {
      ss << " " << id;
    }
    LOG(INFO) << "Num images: " << flow_image_ids.size() << ss.str();
  }

  for (auto image_id : flow_image_ids) {
    FlowImage* flow_image = flow_.MutableFlowImage(image_id);
    AddFlowImageToProblem(flow_image, &problem, &flow_blocks);
  }

  for (auto start = flow_image_ids.begin(), end = ++flow_image_ids.begin();
       end != flow_image_ids.end(); ++start, ++end) {
    FlowImage* flow_image_start = flow_.MutableFlowImage(*start);
    FlowImage* flow_image_end = flow_.MutableFlowImage(*end);

    auto wheel_measurements = wheel_measurements_.find_range(
        flow_image_start->stamp, flow_image_end->stamp);
    VLOG(2) << std::distance(wheel_measurements.first,
                             wheel_measurements.second)
            << " Wheel measurements";

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
  // options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.gradient_tolerance = 1e-4;
  options.function_tolerance = 1e-4;
  options.parameter_tolerance = 1e-4;
  // options.num_threads = 1;
  options.max_num_iterations = 30;

  // Solve
  ceres::Solver::Summary summary;
  //  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = false;
  ceres::Solve(options, &problem, &summary);
  LOG_EVERY_N(INFO, 100) << summary.BriefReport();
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
      double err2 =
          (point_image_proj - block.flow_point_image.point_image.cast<double>())
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
  LOG_EVERY_N(INFO, 100) << "RMSE: "
                         << std::sqrt(all_rmse / std::max(all_n, 1.0))
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
    cv::Mat reprojection_image = debug_image_;
    if (reprojection_image.empty()) {
      cv::cvtColor(*(flow_image->image), reprojection_image,
                   cv::COLOR_GRAY2BGR);
    }

    for (const auto& id_flow_point : flow_image->flow_points) {
      const auto& flow_point = id_flow_point.second;
      FlowPointWorld* flow_point_world =
          flow_.MutableFlowPointWorld(flow_point.id);
      if (flow_point_world->rmse == 0.0) {
        continue;
      }
      Eigen::Vector2d point_image_proj =
          ProjectPointToPixel(camera_model_, flow_image->camera_pose_world *
                                                 flow_point_world->point_world);
      cv::line(reprojection_image, EigenToCvPoint(flow_point.point_image),
               EigenToCvPoint(point_image_proj), flow_.Color(flow_point.id));

      cv::circle(reprojection_image, EigenToCvPoint(point_image_proj), 2,
                 flow_.Color(flow_point.id), -1);
      cv::circle(reprojection_image, EigenToCvPoint(flow_point.point_image), 5,
                 flow_.Color(flow_point.id));
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
    debug_image_ = reprojection_image;
  }
}

void VisualOdometer::DumpFlowPointsWorld(std::string ply_path) {
  std::vector<Eigen::Vector3d> points;
  for (auto it : flow_.FlowPointsWorld()) {
    if (it.second.image_ids.size() >= 5) {
      points.push_back(it.second.point_world);
    }
  }
  SavePly(ply_path, points);
}

void VisualOdometer::SetGoal() {
  if (flow_.LastImageId() < 10) {
    return;
  }
  goal_image_id_ = flow_.LastImageId();
  camera_pose_base_goal_ = base_pose_camera_.inverse();
}

void VisualOdometer::AdjustGoalAngle(double theta) {
  if (!goal_image_id_) {
    return;
  }
  camera_pose_base_goal_ = camera_pose_base_goal_ * Sophus::SE3d::rotZ(theta);
}

}  // namespace calibration
}  // namespace farm_ng
