#include "farm_ng/calibration/intrinsic_calibrator.h"

#include <ceres/ceres.h>
#include <opencv2/highgui.hpp>  // TODO(ethanrublee) remove
#include <opencv2/imgproc.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/create_video_dataset.pb.h"
#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/pose_graph.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/video_streamer.h"

#include "farm_ng/calibration/local_parameterization.h"

namespace farm_ng {
namespace calibration {

using perception::ApriltagRig;
using perception::ApriltagRigIdMap;
using perception::CameraModel;
using perception::FrameNameNumber;
using perception::Image;
using perception::ImageLoader;
using perception::NamedSE3Pose;
using perception::PointsImage;
using perception::PointsTag;
using perception::PoseEdge;
using perception::PoseGraph;
using perception::SE3Map;
using Sophus::SE3d;

struct IntrinsicCostFunctor {
  IntrinsicCostFunctor(const CameraModel& camera,
                       std::array<Eigen::Vector3d, 4> points_tag,
                       std::array<Eigen::Vector2d, 4> points_image,
                       SE3Map camera_pose_tag_rig, SE3Map tag_rig_pose_tag)
      : camera_(camera),
        points_tag_(points_tag),
        points_image_(points_image),
        camera_pose_tag_rig_(camera_pose_tag_rig),
        tag_rig_pose_tag_(tag_rig_pose_tag) {}

  template <class T>
  Eigen::Matrix<T, 4, 2> Project(T const* const raw_camera_model,
                                 T const* const raw_camera_pose_tag_rig,
                                 T const* const raw_tag_rig_pose_tag) const {
    auto camera_pose_tag_rig =
        camera_pose_tag_rig_.Map(raw_camera_pose_tag_rig);
    auto tag_rig_pose_tag = tag_rig_pose_tag_.Map(raw_tag_rig_pose_tag);

    Sophus::SE3<T> camera_pose_tag = camera_pose_tag_rig * tag_rig_pose_tag;

    perception::CameraModelJetMap<T> camera_model(raw_camera_model, camera_);
    Eigen::Matrix<T, 4, 2> points_image;
    for (int i = 0; i < 4; ++i) {
      points_image.row(i) = ProjectPointToPixel(
          camera_model, camera_pose_tag * points_tag_[i].cast<T>());
    }
    return points_image;
  }

  template <class T>
  bool operator()(T const* const raw_camera_model,
                  T const* const raw_camera_pose_tag_rig,
                  T const* const raw_tag_rig_pose_tag, T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 4, 2>> residuals(raw_residuals);

    Eigen::Matrix<T, 4, 2> points_image = Project(
        raw_camera_model, raw_camera_pose_tag_rig, raw_tag_rig_pose_tag);

    for (int i = 0; i < 4; ++i) {
      residuals.row(i) =
          points_image_[i].cast<T>() - points_image.row(i).transpose();
    }
    return true;
  }
  CameraModel camera_;
  std::array<Eigen::Vector3d, 4> points_tag_;
  std::array<Eigen::Vector2d, 4> points_image_;
  SE3Map camera_pose_tag_rig_;
  SE3Map tag_rig_pose_tag_;
};

std::tuple<PoseGraph, ApriltagRigIdMap> InitialPoseGraphFromModel(
    IntrinsicModel* model) {
  PoseGraph pose_graph;
  ApriltagRigIdMap id_map;
  // Add all of the frames from the given apriltag rigs.
  for (const auto& apriltag_rig : model->apriltag_rigs()) {
    id_map.AddRig(apriltag_rig);
    for (const ApriltagRig::Node& node : apriltag_rig.nodes()) {
      pose_graph.AddPose(node.pose());
    }
  }
  model->clear_camera_poses_rig();

  // Now compute an average camera_pose_rig for each view.
  // This is accomplished by composing the per tag camera_pose_tag with the
  // rig's tag_pose_rig, and using PoseGraph::AverageAPoseB() to compute a
  // reasonable intitialization. This is dependent on the ApriltagDetection
  // having a good initial guess for the pose - which is dependent on intrinsic
  // parameters initialized.  During capture, either a good initial guess must
  // be used, for example based on lens specifications, or a camera_model
  // produced by perception::DefaultCameraModel. TODO(ethanrublee) If this
  // function produces nans, may need to revisit initialization scheme.  In
  // practice, even a very bad pose initialization is expected to converge (does
  // Z+ have the right direction, acceptable in plane rotation?)
  for (int i = 0; i < model->detections_size(); ++i) {
    const auto& detections = model->detections(i);
    std::string camera_frame_name =
        FrameNameNumber(model->camera_model().frame_name(), i);
    for (const auto& detection : detections.detections()) {
      auto frame_names = id_map.GetFrameNames(detection.id());
      if (!frame_names) {
        continue;
      }
      // NOTE this pose is possibly suspect, if the camera_model initialization
      // is too far off.
      SE3d camera_pose_tag;
      perception::ProtoToSophus(detection.pose().a_pose_b(), &camera_pose_tag);

      // TODO(rublee) handle NamedPose similar to SE3Map with a helper for this
      // logic.  Here we just are inverting if the frame convention serialized
      // is opposite what we expect.
      if (detection.pose().frame_b() == model->camera_model().frame_name()) {
        camera_pose_tag = camera_pose_tag.inverse();
      } else {
        CHECK_EQ(detection.pose().frame_a(),
                 model->camera_model().frame_name());
      }
      std::optional<SE3d> tag_pose_rig = pose_graph.AverageAPoseB(
          frame_names->tag_frame, frame_names->rig_frame);
      if (!tag_pose_rig) {
        continue;
      }
      SE3d camera_pose_rig = camera_pose_tag * (*tag_pose_rig);
      pose_graph.AddPose(camera_frame_name, frame_names->rig_frame,
                         camera_pose_rig);
    }
    for (const auto& apriltag_rig : model->apriltag_rigs()) {
      auto camera_pose_rig =
          pose_graph.AverageAPoseB(camera_frame_name, apriltag_rig.name());
      if (camera_pose_rig) {
        perception::SophusToProto(
            *camera_pose_rig,
            detections.detections(0).pose().a_pose_b().stamp(),
            camera_frame_name, apriltag_rig.name(),
            model->add_camera_poses_rig());
      }
    }
  }
  return {pose_graph, id_map};
}

std::tuple<PoseGraph, ApriltagRigIdMap> PoseGraphFromModel(
    const IntrinsicModel& model) {
  PoseGraph pose_graph;
  ApriltagRigIdMap id_map;

  for (const ApriltagRig& rig : model.apriltag_rigs()) {
    id_map.AddRig(rig);
    for (const ApriltagRig::Node& node : rig.nodes()) {
      pose_graph.AddPose(node.pose());
    }
  }
  for (const NamedSE3Pose& camera_pose_rig : model.camera_poses_rig()) {
    pose_graph.AddPose(camera_pose_rig);
  }
  return {pose_graph, id_map};
}

void ModelError(IntrinsicModel* model, bool disable_reprojection_images) {
  model->set_rmse(0.0);
  model->clear_reprojection_images();
  model->clear_tag_stats();
  PoseGraph pose_graph;
  ApriltagRigIdMap id_map;
  std::tie(pose_graph, id_map) = PoseGraphFromModel(*model);

  double total_rmse = 0.0;
  double total_count = 0.0;

  std::map<int, ApriltagRigTagStats> tag_stats;

  ImageLoader image_loader;

  for (int i = 0; i < model->detections_size(); ++i) {
    std::string camera_frame =
        FrameNameNumber(model->camera_model().frame_name(), i);

    cv::Mat image;
    if (!disable_reprojection_images) {
      image = image_loader.LoadImage(model->detections(i).image());
      if (image.channels() == 1) {
        cv::Mat color;
        cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
        image = color;
      }
    }

    for (const auto& detection : model->detections(i).detections()) {
      auto tag_frames = id_map.GetFrameNames(detection.id());
      if (!tag_frames) {
        LOG(INFO) << "Unknown tag id: " << detection.id();
        continue;
      }
      auto camera_pose_rig =
          pose_graph.AverageAPoseB(camera_frame, tag_frames->rig_frame);
      if (!camera_pose_rig) {
        LOG(INFO) << "No pose estimated for " << camera_frame << " <- "
                  << tag_frames->rig_frame;
        continue;
      }
      auto rig_pose_tag = pose_graph.AverageAPoseB(tag_frames->rig_frame,
                                                   tag_frames->tag_frame);
      CHECK(rig_pose_tag);

      SE3d camera_pose_tag = (*camera_pose_rig) * (*rig_pose_tag);
      auto points_image = PointsImage(detection);
      auto points_tag = PointsTag(detection);
      double tag_rmse = 0;
      for (int i = 0; i < 4; ++i) {
        if (!disable_reprojection_images) {
          cv::circle(image, cv::Point(points_image[i].x(), points_image[i].y()),
                     5, cv::Scalar(255, 0, 0));
        }
        Eigen::Vector2d point_image_proj = perception::ProjectPointToPixel(
            model->camera_model(), camera_pose_tag * points_tag[i]);

        double rmse = (point_image_proj - points_image[i]).squaredNorm();
        tag_rmse += rmse;
        if (!disable_reprojection_images) {
          cv::circle(image,
                     cv::Point(point_image_proj.x(), point_image_proj.y()), 3,
                     cv::Scalar(0, 0, 255), -1);
        }
      }
      total_rmse += tag_rmse;
      total_count += 8;  // 4*2 residuals

      ApriltagRigTagStats& stats = tag_stats[detection.id()];
      stats.set_tag_id(detection.id());
      stats.set_n_frames(stats.n_frames() + 1);
      stats.set_tag_rig_rmse(stats.tag_rig_rmse() + tag_rmse / 8);
      PerImageRmse* image_rmse = stats.add_per_image_rmse();
      image_rmse->set_rmse(std::sqrt(tag_rmse / 8));
      image_rmse->set_frame_number(i);
      image_rmse->set_camera_name(model->camera_model().frame_name());

      if (!disable_reprojection_images) {
        cv::Point dc(detection.c().x(), detection.c().y());
        std::string id_str = std::to_string(detection.id());

        int baseline;
        cv::Size text_size =

            cv::getTextSize(id_str, cv::FONT_HERSHEY_PLAIN, 1.0, 1, &baseline);
        cv::rectangle(image,
                      cv::Rect(dc + cv::Point(-1, 1),
                               dc + cv::Point(text_size.width + 1,
                                              -text_size.height - 1)),

                      cv::Scalar(0, 0, 0), -1);

        cv::putText(image, id_str, dc, cv::FONT_HERSHEY_PLAIN, 1.0,
                    cv::Scalar(125, 255, 125));
      }
    }
    if (!disable_reprojection_images) {
      Image& reprojection_image = *model->add_reprojection_images();
      reprojection_image.mutable_camera_model()->CopyFrom(
          model->camera_model());
      auto resource_path = core::GetUniqueArchiveResource(
          FrameNameNumber(
              "reprojection-" + SolverStatus_Name(model->solver_status()), i),
          "jpg", "image/jpg");
      reprojection_image.mutable_resource()->CopyFrom(resource_path.first);
      LOG(INFO) << resource_path.second.string();
      CHECK(cv::imwrite(resource_path.second.string(), image))
          << "Could not write: " << resource_path.second;
    }
  }
  for (auto& stats : tag_stats) {
    stats.second.set_tag_rig_rmse(
        std::sqrt(stats.second.tag_rig_rmse() / stats.second.n_frames()));
    auto debug_stats = stats.second;
    debug_stats.clear_per_image_rmse();
    LOG(INFO) << debug_stats.DebugString();
    model->add_tag_stats()->CopyFrom(stats.second);
  }
  model->set_rmse(std::sqrt(total_rmse / total_count));
  LOG(INFO) << "model rmse (pixels): " << model->rmse();
}

IntrinsicModel SolveIntrinsicsModel(IntrinsicModel model) {
  perception::CameraModelJetMap<double> camera_model_param(
      model.camera_model());
  PoseGraph pose_graph;
  ApriltagRigIdMap id_map;
  std::tie(pose_graph, id_map) = InitialPoseGraphFromModel(&model);

  ceres::Problem problem;
  problem.AddParameterBlock(camera_model_param.data(),
                            camera_model_param.data_size(), nullptr);
  for (PoseEdge* pose_edge : pose_graph.MutablePoseEdges()) {
    // LOG(INFO) << *pose_edge;
    problem.AddParameterBlock(pose_edge->GetAPoseB().data(),
                              SE3d::num_parameters,
                              new LocalParameterizationSE3);

    if (pose_edge->frame_a.find(model.camera_model().frame_name()) ==
            std::string::npos &&
        pose_edge->frame_b.find(model.camera_model().frame_name()) ==
            std::string::npos) {
      LOG(INFO) << "Setting constant: " << *pose_edge;
      problem.SetParameterBlockConstant(pose_edge->GetAPoseB().data());
    }
  }

  for (int i = 0; i < model.detections_size(); ++i) {
    std::string camera_frame =
        FrameNameNumber(model.camera_model().frame_name(), i);
    for (const auto& detection : model.detections(i).detections()) {
      auto frame_names = id_map.GetFrameNames(detection.id());
      if (!frame_names) {
        continue;
      }
      PoseEdge* tag_to_tag_rig = pose_graph.MutablePoseEdge(
          frame_names->tag_frame, frame_names->rig_frame);

      PoseEdge* camera_to_tag_rig =
          pose_graph.MutablePoseEdge(camera_frame, frame_names->rig_frame);

      ceres::CostFunction* cost_function1 = new ceres::AutoDiffCostFunction<
          IntrinsicCostFunctor,
          8,  // 2*4 residuals
          perception::CameraModelJetMap<double>::num_parameters,  // camera
                                                                  // intrinsics
          Sophus::SE3d::num_parameters,  // camera_to_rig
          Sophus::SE3d::num_parameters   // tag_to_rig
          >(new IntrinsicCostFunctor(
          model.camera_model(), PointsTag(detection), PointsImage(detection),
          camera_to_tag_rig->GetAPoseBMap(camera_frame, frame_names->rig_frame),
          tag_to_tag_rig->GetAPoseBMap(frame_names->rig_frame,
                                       frame_names->tag_frame)));
      problem.AddResidualBlock(cost_function1, nullptr,
                               camera_model_param.data(),
                               camera_to_tag_rig->GetAPoseB().data(),
                               tag_to_tag_rig->GetAPoseB().data());
    }
  }

  // Set solver options (precision / method)
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
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
  if (summary.IsSolutionUsable()) {
    LOG(INFO) << " Initial model: " << model.camera_model().ShortDebugString();
    LOG(INFO) << " Solved model: "
              << camera_model_param.GetCameraModel().ShortDebugString();
    model.mutable_camera_model()->CopyFrom(camera_model_param.GetCameraModel());
    pose_graph.UpdateNamedSE3Poses(model.mutable_camera_poses_rig());
    model.set_solver_status(SolverStatus::SOLVER_STATUS_CONVERGED);
  } else {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_FAILED);
  }

  return model;
}

IntrinsicModel InitialIntrinsicModelFromConfig(
    const CalibrateIntrinsicsConfiguration& config) {
  // TODO add support for
  auto dataset_result =
      core::ReadProtobufFromResource<perception::CreateVideoDatasetResult>(
          config.video_dataset());

  IntrinsicModel intrinsic_model;
  intrinsic_model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);
  intrinsic_model.set_rmse(0);

  for (auto rig_resource : dataset_result.configuration().apriltag_rigs()) {
    intrinsic_model.add_apriltag_rigs()->CopyFrom(
        core::ReadProtobufFromResource<perception::ApriltagRig>(rig_resource));
  }

  core::EventLogReader log_reader(dataset_result.dataset());
  perception::ApriltagsFilter filter(perception::ApriltagsFilter::FILTER_NOVEL);
    int window_size = 7;
    int steady_count =5;
    if(config.has_steady_count()) {
      steady_count = config.steady_count().value();
    }
    if(config.has_steady_window_size()) {
      window_size = config.steady_window_size().value();
    }

  while (true) {
    core::Event event;
    try {
      event = log_reader.ReadNext();
    } catch (std::runtime_error& e) {
      break;
    }
    perception::ApriltagDetections detections;
    if (event.data().UnpackTo(&detections)) {
      if (detections.image().camera_model().frame_name() !=
          config.camera_name()) {
        continue;
      }
      if (config.filter_stable_tags() &&
          !filter.AddApriltags(detections, steady_count, window_size)) {
        continue;
      }
      intrinsic_model.add_detections()->CopyFrom(detections);
      LOG(INFO) << "N detections: " << intrinsic_model.detections_size();
      if (!intrinsic_model.camera_model().image_height()) {
        LOG(INFO) << "Initial model: "
                  << detections.image().camera_model().ShortDebugString();
        intrinsic_model.mutable_camera_model()->CopyFrom(
            detections.image().camera_model());
        if (config.distortion_model() !=
            CameraModel::DISTORTION_MODEL_UNSPECIFIED) {
          LOG(INFO) << "Solving for distortion model: "
                    << CameraModel::DistortionModel_Name(
                           config.distortion_model());
          intrinsic_model.mutable_camera_model()->set_distortion_model(
              config.distortion_model());
        }
      } else {
        CHECK_EQ(intrinsic_model.camera_model().image_height(),
                 detections.image().camera_model().image_height());
        CHECK_EQ(intrinsic_model.camera_model().image_width(),
                 detections.image().camera_model().image_width());
      }
    }
  }

  return intrinsic_model;
}
}  // namespace calibration
}  // namespace farm_ng
