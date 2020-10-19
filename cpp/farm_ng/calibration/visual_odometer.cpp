#include "farm_ng/calibration/visual_odometer.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include <ceres/ceres.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/sophus_protobuf.h"

#include "farm_ng/calibration/camera_model.h"
#include "farm_ng/calibration/eigen_cv.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"

namespace farm_ng {
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

void VisualOdometer::AddImage(cv::Mat image,
                              google::protobuf::Timestamp stamp) {
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

void VisualOdometer::AddFlowBlockToProblem(ceres::Problem* problem,
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

void VisualOdometer::AddFlowImageToProblem(FlowImage* flow_image,
                                           ceres::Problem* problem,
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
    if (flow_blocks->size() < 250 || flow_blocks->count(flow_point_world->id)) {
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

    cv::cvtColor(*(flow_image->image), reprojection_image, cv::COLOR_GRAY2BGR);

    for (const auto& id_flow_point : flow_image->flow_points) {
      const auto& flow_point = id_flow_point.second;
      FlowPointWorld* flow_point_world =
          flow_.MutableFlowPointWorld(flow_point.flow_point_id);
      if (flow_point_world->rmse == 0.0) {
        continue;
      }
      Eigen::Vector2d point_image_proj =
          ProjectPointToPixel(camera_model_, flow_image->camera_pose_world *
                                                 flow_point_world->point_world);
      cv::line(reprojection_image, EigenToCvPoint(flow_point.image_coordinates),
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

}  // namespace farm_ng
