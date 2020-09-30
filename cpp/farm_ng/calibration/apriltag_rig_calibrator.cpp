#include "farm_ng/calibration/apriltag_rig_calibrator.h"

#include <ceres/ceres.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sophus/average.hpp>

#include "farm_ng/calibration/apriltag.h"
#include "farm_ng/calibration/camera_model.h"
#include "farm_ng/calibration/local_parameterization_se3.h"
#include "farm_ng/calibration/pose_utils.h"

#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::ApriltagRig;

Sophus::optional<SE3d> CameraPoseRigRootInit(
    const std::unordered_map<int, SE3d>& tag_poses_root,
    const std::unordered_map<int, SE3d>& camera_poses_tag) {
  std::vector<SE3d> camera_poses_root;
  for (const auto& id_camera_pose_tag : camera_poses_tag) {
    int tag_id = id_camera_pose_tag.first;
    const SE3d& camera_pose_tag = id_camera_pose_tag.second;
    auto tag_pose_root = tag_poses_root.find(tag_id);
    if (tag_pose_root == tag_poses_root.end()) {
      continue;
    }
    camera_poses_root.push_back(camera_pose_tag * tag_pose_root->second);
  }
  if (camera_poses_root.empty()) {
    return Sophus::optional<SE3d>();
  }
  return Sophus::average(camera_poses_root);
}

struct CameraApriltagRigCostFunctor {
  CameraApriltagRigCostFunctor(const CameraModel& camera,
                               std::array<Eigen::Vector3d, 4> points_tag,
                               std::array<Eigen::Vector2d, 4> points_image)
      : camera_(camera), points_tag_(points_tag), points_image_(points_image) {}

  template <class T>
  bool operator()(T const* const raw_camera_pose_root,
                  T const* const raw_tag_pose_root, T* raw_residuals) const {
    Eigen::Map<Sophus::SE3<T> const> const camera_pose_root(
        raw_camera_pose_root);
    Eigen::Map<Sophus::SE3<T> const> const tag_pose_root(raw_tag_pose_root);
    Eigen::Map<Eigen::Matrix<T, 4, 2>> residuals(raw_residuals);

    Sophus::SE3<T> camera_pose_tag = camera_pose_root * tag_pose_root.inverse();

    for (int i = 0; i < 4; ++i) {
      residuals.row(i) =
          points_image_[i].cast<T>() -
          ProjectPointToPixel(camera_,
                              camera_pose_tag * points_tag_[i].cast<T>());
    }
    return true;
  }
  CameraModel camera_;
  std::array<Eigen::Vector3d, 4> points_tag_;
  std::array<Eigen::Vector2d, 4> points_image_;
};

void ApriltagRigModel::ToMonocularApriltagRigModel(
    MonocularApriltagRigModel* rig) const {
  rig->Clear();
  rig->mutable_rig()->set_name(rig_name);
  rig->mutable_rig()->set_root_tag_id(root_id);

  for (auto id_tag_pose_root : tag_pose_root) {
    int id = id_tag_pose_root.first;
    const SE3d& tag_pose_root = id_tag_pose_root.second;

    ApriltagRig::Node* node = rig->mutable_rig()->add_nodes();
    node->set_id(id);
    node->set_frame_name(FrameRigTag(rig_name, id));
    NamedSE3Pose* pose = node->mutable_pose();
    pose->set_frame_a(FrameRigTag(rig_name, id));
    pose->set_frame_b(FrameRigTag(rig_name, root_id));
    SophusToProto(tag_pose_root, pose->mutable_a_pose_b());
    node->set_tag_size(tag_size.at(id));
    for (const auto& v : points_tag.at(id)) {
      EigenToProto(v, node->add_points_tag());
    }
  }
  rig->set_solver_status(status);
  rig->set_rmse(rmse);
  for (const auto& stats : per_tag_stats) {
    rig->add_tag_stats()->CopyFrom(stats.second);
  }
  rig->set_camera_frame_name(camera_frame_name);
  for (const auto& detections : all_detections) {
    rig->add_detections()->CopyFrom(detections);
  }
  for (size_t frame_n = 0; frame_n < camera_poses_root.size(); ++frame_n) {
    auto o_camera_pose_root = camera_poses_root[frame_n];
    if (o_camera_pose_root) {
      NamedSE3Pose* pose = rig->add_camera_poses_rig();
      pose->set_frame_a(FrameNameNumber(camera_frame_name, frame_n));
      pose->set_frame_b(FrameRigTag(rig_name, root_id));
      SophusToProto(*o_camera_pose_root, pose->mutable_a_pose_b());
    }
  }

  for (const auto& image : reprojection_images) {
    rig->add_reprojection_images()->CopyFrom(image);
  }
}

void ModelError(ApriltagRigModel& model) {
  double all_sum_error = 0;
  double all_error_count = 0;
  model.per_tag_stats.clear();

  for (size_t frame_n = 0; frame_n < model.camera_poses_root.size();
       ++frame_n) {
    auto& o_camera_pose_root = model.camera_poses_root[frame_n];
    if (!o_camera_pose_root) {
      continue;
    }

    const auto& detections = model.all_detections[frame_n];
    all_error_count += detections.detections_size() * 4;
    cv::Mat image = cv::imread(
        (GetArchiveRoot() / detections.image().resource().path()).string(),
        cv::IMREAD_COLOR);

    // compute reprojection error.
    for (const auto& detection : detections.detections()) {
      auto& per_tag_stats = model.per_tag_stats[detection.id()];
      per_tag_stats.set_tag_id(detection.id());
      per_tag_stats.set_n_frames(per_tag_stats.n_frames() + 1);

      auto points_tag = PointsTag(detection);
      auto points_image = PointsImage(detection);
      SE3d camera_pose_tag = (*o_camera_pose_root) *
                             model.tag_pose_root.at(detection.id()).inverse();

      double tag_error = 0;
      for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d rp = ProjectPointToPixel(
            detections.image().camera_model(), camera_pose_tag * points_tag[i]);

        cv::circle(image, cv::Point(points_image[i].x(), points_image[i].y()),
                   5, cv::Scalar(255, 0, 0));

        double norm = (points_image[i] - rp).squaredNorm();
        tag_error += norm / 4;
        all_sum_error += norm;
      }
      per_tag_stats.set_tag_rig_rmse(per_tag_stats.tag_rig_rmse() + tag_error);

      auto* image_rmse = per_tag_stats.add_per_image_rmse();
      image_rmse->set_frame_number(frame_n);
      image_rmse->set_rmse(std::sqrt(tag_error));
    }
    // Reproject all tag points for visualization
    for (auto tag_id_points_tag : model.points_tag) {
      int tag_id = tag_id_points_tag.first;
      auto points_tag = tag_id_points_tag.second;
      SE3d camera_pose_tag =
          (*o_camera_pose_root) * model.tag_pose_root.at(tag_id).inverse();
      for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d point_camera = camera_pose_tag * points_tag[i];
        if (point_camera.z() < 0.1) {  // point not in field of view.
          continue;
        }
        Eigen::Vector2d rp = ProjectPointToPixel(
            detections.image().camera_model(), point_camera);

        cv::circle(image, cv::Point(rp.x(), rp.y()), 3, cv::Scalar(0, 0, 255),
                   -1);
      }
    }
    Image reprojection_image;
    reprojection_image.CopyFrom(detections.image());
    auto resource_path = GetUniqueResource(
        FrameNameNumber("reprojection", frame_n), "png", "image/png");
    reprojection_image.mutable_resource()->CopyFrom(resource_path.first);

    CHECK(cv::imwrite(resource_path.second.string(), image))
        << "Could not write: " << resource_path.second;
    model.reprojection_images.push_back(reprojection_image);
  }
  model.rmse = std::sqrt(all_sum_error / all_error_count);
  LOG(INFO) << "RMSE for all frames: " << model.rmse;
  for (auto& it : model.per_tag_stats) {
    it.second.set_tag_rig_rmse(
        std::sqrt(it.second.tag_rig_rmse() / it.second.n_frames()));
    LOG(INFO) << "tag: " << it.second.tag_id()
              << " n_frames: " << it.second.n_frames()
              << " rig_rmse: " << it.second.tag_rig_rmse();
  }
}

ApriltagRigCalibrator::ApriltagRigCalibrator(
    EventBus* bus, const CalibratorCommand::ApriltagRigStart& rig_start)
    : bus_(bus), root_id_(0) {
  if (rig_start.tag_ids_size() > 0) {
    root_id_ = rig_start.tag_ids().Get(0);
  }
  for (auto id : rig_start.tag_ids()) {
    ids_.insert(id);
  }
}

void ApriltagRigCalibrator::PoseInit(
    const std::vector<std::unordered_map<int, SE3d>>& frames,
    std::unordered_map<int, SE3d>& tag_mean_pose_root,
    std::vector<Sophus::optional<SE3d>>& camera_poses_root) {
  CHECK_EQ(frames.size(), camera_poses_root.size());
  CHECK(tag_mean_pose_root.count(root_id_));
  int starting_size = tag_mean_pose_root.size();

  std::unordered_map<int, std::vector<SE3d>> tag_poses_root;
  for (size_t frame_n = 0; frame_n < frames.size(); ++frame_n) {
    const auto& camera_pose_tags = frames[frame_n];
    Sophus::optional<SE3d>& camera_pose_root = camera_poses_root[frame_n];
    std::vector<SE3d> view_poses_root;
    for (const auto& id_camera_pose_id : camera_pose_tags) {
      int id = id_camera_pose_id.first;
      const SE3d& camera_pose_id = id_camera_pose_id.second;
      if (tag_mean_pose_root.count(id)) {
        view_poses_root.push_back(camera_pose_id * tag_mean_pose_root.at(id));
      }
    }
    if (!view_poses_root.empty()) {
      auto o_camera_pose_root = Sophus::average(view_poses_root);
      if (o_camera_pose_root) {
        camera_pose_root = *o_camera_pose_root;
      } else {
        LOG(WARNING) << "failed to average camera pose for frame: " << frame_n;
      }
    }

    if (camera_pose_root) {
      for (const auto& id_camera_pose_id : camera_pose_tags) {
        // skip the root, as its already added, and is the identity.
        if (id_camera_pose_id.first == root_id_) {
          continue;
        }
        int id = id_camera_pose_id.first;
        const SE3d& camera_pose_id = id_camera_pose_id.second;
        tag_poses_root[id].push_back(camera_pose_id.inverse() *
                                     (*camera_pose_root));
      }
    }
  }
  for (const auto& id_tag_poses_root : tag_poses_root) {
    auto o_tag_pose_root = Sophus::average(id_tag_poses_root.second);
    if (o_tag_pose_root) {
      tag_mean_pose_root[id_tag_poses_root.first] = *o_tag_pose_root;

      VLOG(2) << "Average SE3 pose " << id_tag_poses_root.first << "_pose_"
              << root_id_
              << " = t:" << o_tag_pose_root->translation().transpose()
              << " q:" << o_tag_pose_root->unit_quaternion().vec().transpose();
    } else {
      VLOG(2) << "Could not find average SE3 pose for "
              << id_tag_poses_root.first << "_pose_" << root_id_;
    }
  }
  int ending_size = tag_mean_pose_root.size();
  if (ending_size != starting_size) {
    PoseInit(frames, tag_mean_pose_root, camera_poses_root);
  }
}

ApriltagRigModel ApriltagRigCalibrator::PoseInitialization() {
  std::vector<std::unordered_map<int, SE3d>> frames;

  std::string camera_frame_name;

  std::unordered_map<int, double> tag_size;
  std::unordered_map<int, std::array<Eigen::Vector3d, 4>> points_tag;

  for (const auto& detections : all_detections_) {
    std::unordered_map<int, SE3d> camera_pose_tags;
    if (camera_frame_name.empty()) {
      camera_frame_name = detections.image().camera_model().frame_name();
    }
    CHECK_EQ(camera_frame_name, detections.image().camera_model().frame_name());
    for (const auto& detection : detections.detections()) {
      CHECK(camera_pose_tags.count(detection.id()) == 0)
          << "Duplicate detection tag id: " << detection.id()
          << " frame: " << frames.size();
      if (tag_size.count(detection.id())) {
        CHECK_EQ(tag_size.at(detection.id()), detection.tag_size());
      } else {
        tag_size[detection.id()] = detection.tag_size();
        points_tag[detection.id()] = PointsTag(detection);
      }
      SE3d a_pose_b;
      ProtoToSophus(detection.pose().a_pose_b(), &a_pose_b);
      if (StartsWith(detection.pose().frame_a(), "tag/")) {
        CHECK_EQ(camera_frame_name, detection.pose().frame_b());
        camera_pose_tags[detection.id()] = a_pose_b.inverse();
      } else if (StartsWith(detection.pose().frame_b(), "tag/")) {
        CHECK_EQ(camera_frame_name, detection.pose().frame_a());
        camera_pose_tags[detection.id()] = a_pose_b;
      } else {
        CHECK(false) << "Malformed apriltag : " << detection.ShortDebugString();
      }
    }
    frames.push_back(camera_pose_tags);
  }

  std::unordered_map<int, SE3d> tag_mean_pose_root;
  tag_mean_pose_root[root_id_] = SE3d::transX(0);
  std::vector<Sophus::optional<SE3d>> camera_poses_root(frames.size());

  PoseInit(frames, tag_mean_pose_root, camera_poses_root);

  ApriltagRigModel model({root_id_,
                          rig_name_,
                          camera_frame_name,
                          all_detections_,
                          tag_mean_pose_root,
                          tag_size,
                          points_tag,
                          camera_poses_root,
                          {},
                          SolverStatus::SOLVER_STATUS_INITIAL,
                          {},
                          0.0});
  ModelError(model);
  return model;
}

void ApriltagRigCalibrator::AddFrame(ApriltagDetections detections) {
  for (auto it = detections.mutable_detections()->begin();
       it != detections.mutable_detections()->end();) {
    if (it->tag_size() == 0.0) {
      it->set_tag_size(0.16);
    }
    if (ids_.count(it->id()) == 0) {
      it = detections.mutable_detections()->erase(it);
    } else {
      ++it;
    }
  }
  all_detections_.push_back(std::move(detections));
}

int ApriltagRigCalibrator::NumFrames() const {
  return static_cast<int>(all_detections_.size());
}

bool Solve(ApriltagRigModel& model) {
  ceres::Problem problem;

  for (auto& id_tag_pose_root : model.tag_pose_root) {
    int tag_id = id_tag_pose_root.first;
    SE3d& tag_pose_root = id_tag_pose_root.second;
    // Specify local update rule for our parameter
    problem.AddParameterBlock(tag_pose_root.data(), SE3d::num_parameters,
                              new LocalParameterizationSE3);
    if (tag_id == model.root_id) {
      problem.SetParameterBlockConstant(tag_pose_root.data());
    }
  }

  for (size_t frame_n = 0; frame_n < model.camera_poses_root.size();
       ++frame_n) {
    auto& o_camera_pose_root = model.camera_poses_root[frame_n];
    if (!o_camera_pose_root) {
      continue;
    }
    // Specify local update rule for our parameter
    problem.AddParameterBlock(o_camera_pose_root->data(), SE3d::num_parameters,
                              new LocalParameterizationSE3);

    const auto& detections = model.all_detections[frame_n];
    for (const auto& detection : detections.detections()) {
      ceres::CostFunction* cost_function1 =
          new ceres::AutoDiffCostFunction<CameraApriltagRigCostFunctor, 8,
                                          Sophus::SE3d::num_parameters,
                                          Sophus::SE3d::num_parameters>(
              new CameraApriltagRigCostFunctor(
                  detections.image().camera_model(), PointsTag(detection),
                  PointsImage(detection)));
      problem.AddResidualBlock(cost_function1, new ceres::HuberLoss(1.0),
                               o_camera_pose_root->data(),
                               model.tag_pose_root.at(detection.id()).data());
    }
  }

  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.gradient_tolerance = 1e-18;
  options.function_tolerance = 1e-18;
  options.parameter_tolerance = 1e-18;
  options.max_num_iterations = 2000;

  // Solve
  ceres::Solver::Summary summary;
  options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  options.minimizer_progress_to_stdout = true;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport() << std::endl;
  if (summary.termination_type == ceres::CONVERGENCE) {
    model.status = SolverStatus::SOLVER_STATUS_CONVERGED;
  } else {
    model.status = SolverStatus::SOLVER_STATUS_FAILED;
  }

  LOG(INFO) << "root mean of residual error: "
            << std::sqrt(summary.final_cost / summary.num_residuals);
  ModelError(model);

  return true;
}

Sophus::optional<NamedSE3Pose> EstimateCameraPoseRig(
    const ApriltagRig& rig, const ApriltagDetections& detections) {
  std::unordered_map<int, Sophus::SE3d> id_tag_poses_root;
  for (const ApriltagRig::Node& node : rig.nodes()) {
    Sophus::SE3d tag_pose_root;
    ProtoToSophus(node.pose().a_pose_b(), &tag_pose_root);
    CHECK_EQ(node.pose().frame_a(), FrameRigTag(rig.name(), node.id()));
    CHECK_EQ(node.pose().frame_b(), FrameRigTag(rig.name(), rig.root_tag_id()));
    id_tag_poses_root.insert(std::make_pair(node.id(), tag_pose_root));
  }
  std::unordered_map<int, Sophus::SE3d> id_camera_poses_tag;
  auto camera_model = detections.image().camera_model();
  for (const auto& detection : detections.detections()) {
    SE3d camera_pose_tag;
    ProtoToSophus(detection.pose().a_pose_b(), &camera_pose_tag);
    if (StartsWith(detection.pose().frame_a(), "tag/")) {
      CHECK_EQ(camera_model.frame_name(), detection.pose().frame_b());
      camera_pose_tag = camera_pose_tag.inverse();
    } else if (StartsWith(detection.pose().frame_b(), "tag/")) {
      CHECK_EQ(camera_model.frame_name(), detection.pose().frame_a());
    } else {
      CHECK(false) << "Malformed apriltag : " << detection.ShortDebugString();
    }
    id_camera_poses_tag[detection.id()] = camera_pose_tag;
  }
  auto o_camera_pose_root =
      CameraPoseRigRootInit(id_tag_poses_root, id_camera_poses_tag);
  if (!o_camera_pose_root) {
    return Sophus::optional<NamedSE3Pose>();
  }

  ceres::Problem problem;
  problem.AddParameterBlock(o_camera_pose_root->data(), SE3d::num_parameters,
                            new LocalParameterizationSE3);

  for (auto& id_tag_pose_root : id_tag_poses_root) {
    int tag_id = id_tag_pose_root.first;
    SE3d& tag_pose_root = id_tag_pose_root.second;
    // Specify local update rule for our parameter
    problem.AddParameterBlock(tag_pose_root.data(), SE3d::num_parameters,
                              new LocalParameterizationSE3);
    problem.SetParameterBlockConstant(tag_pose_root.data());
  }

  for (const auto& detection : detections.detections()) {
    ceres::CostFunction* cost_function1 =
        new ceres::AutoDiffCostFunction<CameraApriltagRigCostFunctor, 8,
                                        Sophus::SE3d::num_parameters,
                                        Sophus::SE3d::num_parameters>(
            new CameraApriltagRigCostFunctor(camera_model, PointsTag(detection),
                                             PointsImage(detection)));
    problem.AddResidualBlock(cost_function1, new ceres::HuberLoss(1.0),
                             o_camera_pose_root->data(),
                             id_tag_poses_root.at(detection.id()).data());
  }

  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.gradient_tolerance = 1e-18;
  options.function_tolerance = 1e-18;
  options.parameter_tolerance = 1e-18;
  options.max_num_iterations = 2000;

  // Solve
  ceres::Solver::Summary summary;
  // options.logging_type = ceres::PER_MINIMIZER_ITERATION;
  // options.minimizer_progress_to_stdout = false;
  ceres::Solve(options, &problem, &summary);
  // LOG(INFO) << summary.FullReport() << std::endl;
  if (summary.IsSolutionUsable()) {
    NamedSE3Pose camera_pose_rig;
    SophusToProto(*o_camera_pose_root, camera_model.frame_name(),
                  FrameRigTag(rig.name(), rig.root_tag_id()), &camera_pose_rig);
    VLOG(2) << "root mean of residual error: "
            << std::sqrt(summary.final_cost / summary.num_residuals) << " "
            << camera_pose_rig.ShortDebugString();

    return camera_pose_rig;
  } else {
    return Sophus::optional<NamedSE3Pose>();
  }
}

}  // namespace farm_ng
