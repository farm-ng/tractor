#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

#include <map>

#include <ceres/ceres.h>
#include <google/protobuf/util/time_util.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <opencv2/highgui.hpp>  // TODO remove.
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sophus/average.hpp>

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/calibration/camera_rig_apriltag_rig_cost_functor.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/capture_video_dataset.pb.h"
#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/image_utils.h"
#include "farm_ng/perception/pose_graph.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"
#include "farm_ng/perception/time_series.h"

namespace farm_ng {
namespace calibration {
typedef farm_ng::core::Event EventPb;
using farm_ng::core::EventLogReader;
using farm_ng::core::GetUniqueArchiveResource;
using farm_ng::core::ReadProtobufFromResource;
using farm_ng::core::Resource;
using farm_ng::perception::ApriltagDetections;
using farm_ng::perception::ApriltagRig;
using farm_ng::perception::ApriltagsFilter;
using farm_ng::perception::CameraModel;
using farm_ng::perception::PoseEdge;

using farm_ng::perception::ConstructGridImage;
using farm_ng::perception::FrameNameNumber;
using farm_ng::perception::FrameRigTag;
using farm_ng::perception::Image;
using farm_ng::perception::ImageLoader;
using farm_ng::perception::NamedSE3Pose;
using farm_ng::perception::PoseGraph;
using farm_ng::perception::SE3Map;
using farm_ng::perception::SophusToProto;
using farm_ng::perception::TimeSeries;
using Sophus::SE3d;

using farm_ng::core::Event;
using farm_ng::perception::CaptureVideoDatasetResult;
using farm_ng::perception::MultiViewApriltagDetections;
using Sophus::SE3d;

void GetCameraRigPosesTagRig(const MultiViewApriltagRigModel& model,
                             PoseGraph* pose_graph) {
  std::string tag_rig_view_frame = model.apriltag_rig().name() + "/view/";
  std::map<int, SE3d> camera_rig_poses_tag_rig;
  for (const NamedSE3Pose& camera_rig_pose_tag_rig :
       model.camera_rig_poses_apriltag_rig()) {
    CHECK(camera_rig_pose_tag_rig.frame_b().rfind(tag_rig_view_frame) == 0)
        << camera_rig_pose_tag_rig.frame_b()
        << " does not start with: " << tag_rig_view_frame;
    CHECK_EQ(camera_rig_pose_tag_rig.frame_a(), model.camera_rig().name());
    int frame_n = std::stoi(
        camera_rig_pose_tag_rig.frame_b().substr(tag_rig_view_frame.size()));
    CHECK_GE(frame_n, 0);
    CHECK_LT(frame_n, model.multi_view_detections_size());
    pose_graph->AddPose(camera_rig_pose_tag_rig);
  }
}

PoseGraph PoseGraphFromModel(const MultiViewApriltagRigModel& model) {
  PoseGraph pose_graph;
  for (const ApriltagRig::Node& node : model.apriltag_rig().nodes()) {
    pose_graph.AddPose(node.pose());
  }
  for (const NamedSE3Pose& camera_pose_rig :
       model.camera_rig().camera_pose_rig()) {
    pose_graph.AddPose(camera_pose_rig);
  }
  GetCameraRigPosesTagRig(model, &pose_graph);
  return pose_graph;
}
void UpdateModelFromPoseGraph(const PoseGraph& pose_graph,
                              MultiViewApriltagRigModel* model) {
  for (ApriltagRig::Node& node :
       *model->mutable_apriltag_rig()->mutable_nodes()) {
    pose_graph.UpdateNamedSE3Pose(node.mutable_pose());
  }
  pose_graph.UpdateNamedSE3Poses(
      model->mutable_camera_rig()->mutable_camera_pose_rig());
  pose_graph.UpdateNamedSE3Poses(
      model->mutable_camera_rig_poses_apriltag_rig());
}

void ModelError(MultiViewApriltagRigModel* model) {
  model->set_rmse(0.0);
  model->clear_reprojection_images();
  model->clear_tag_stats();
  PoseGraph pose_graph = PoseGraphFromModel(*model);
  std::string root_tag_frame = FrameRigTag(model->apriltag_rig().name(),
                                           model->apriltag_rig().root_tag_id());
  std::string tag_rig_frame = model->apriltag_rig().name();
  std::string root_camera_frame = model->camera_rig().root_camera_name();
  std::string camera_rig_frame = model->camera_rig().name();
  int frame_num = -1;
  double total_rmse = 0.0;
  double total_count = 0.0;

  std::map<int, ApriltagRigTagStats> tag_stats;

  ImageLoader image_loader;

  for (const auto& mv_detections : model->multi_view_detections()) {
    frame_num++;
    std::string tag_rig_view_frame =
        tag_rig_frame + "/view/" + std::to_string(frame_num);

    if (!pose_graph.HasEdge(camera_rig_frame, tag_rig_view_frame)) {
      continue;
    }
    PoseEdge* camera_rig_to_tag_rig_view =
        pose_graph.MutablePoseEdge(camera_rig_frame, tag_rig_view_frame);
    std::vector<cv::Mat> images;
    for (const auto& detections_per_view :
         mv_detections.detections_per_view()) {
      cv::Mat image = image_loader.LoadImage(detections_per_view.image());
      if (image.channels() == 1) {
        cv::Mat color;
        cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
        image = color;
      }
      const auto& camera_model = detections_per_view.image().camera_model();

      std::string camera_frame = camera_model.frame_name();

      PoseEdge* camera_to_camera_rig =
          pose_graph.MutablePoseEdge(camera_frame, camera_rig_frame);

      for (const auto& node : model->apriltag_rig().nodes()) {
        PoseEdge* tag_to_tag_rig =
            pose_graph.MutablePoseEdge(node.frame_name(), tag_rig_frame);

        auto camera_pose_tag =
            camera_to_camera_rig->GetAPoseBMapped(camera_frame,
                                                  camera_rig_frame) *
            camera_rig_to_tag_rig_view->GetAPoseBMapped(camera_rig_frame,
                                                        tag_rig_view_frame) *
            tag_to_tag_rig->GetAPoseBMapped(tag_rig_frame, node.frame_name());
        for (int i = 0; i < 4; ++i) {
          Eigen::Vector3d point_tag(node.points_tag().Get(i).x(),
                                    node.points_tag().Get(i).y(),
                                    node.points_tag().Get(i).z());
          Eigen::Vector3d point_camera = camera_pose_tag * point_tag;
          if (point_camera.z() > 0.001) {
            Eigen::Vector2d rp =
                ProjectPointToPixel(camera_model, point_camera);
            cv::circle(image, cv::Point(rp.x(), rp.y()), 3,
                       cv::Scalar(0, 0, 255), -1);
          }
        }
      }

      if (detections_per_view.detections_size() < 1) {
        images.push_back(image);
        continue;
      }

      for (const auto& detection : detections_per_view.detections()) {
        std::string tag_frame = FrameRigTag(tag_rig_frame, detection.id());
        PoseEdge* tag_to_tag_rig =
            pose_graph.MutablePoseEdge(tag_frame, tag_rig_frame);
        auto points_image = PointsImage(detection);
        CameraRigApriltagRigCostFunctor cost(
            detections_per_view.image().camera_model(), PointsTag(detection),
            points_image,
            camera_to_camera_rig->GetAPoseBMap(camera_frame, camera_rig_frame),
            tag_to_tag_rig->GetAPoseBMap(tag_rig_frame, tag_frame),
            camera_rig_to_tag_rig_view->GetAPoseBMap(camera_rig_frame,
                                                     tag_rig_view_frame));
        Eigen::Matrix<double, 4, 2> residuals;
        CHECK(cost(camera_to_camera_rig->GetAPoseB().data(),
                   tag_to_tag_rig->GetAPoseB().data(),
                   camera_rig_to_tag_rig_view->GetAPoseB().data(),
                   residuals.data()));

        for (int i = 0; i < 4; ++i) {
          cv::circle(image, cv::Point(points_image[i].x(), points_image[i].y()),
                     5, cv::Scalar(255, 0, 0));
        }
        total_rmse += residuals.squaredNorm();
        total_count += 8;
        ApriltagRigTagStats& stats = tag_stats[detection.id()];
        stats.set_tag_id(detection.id());
        stats.set_n_frames(stats.n_frames() + 1);
        stats.set_tag_rig_rmse(stats.tag_rig_rmse() +
                               residuals.squaredNorm() / 8);
        PerImageRmse* image_rmse = stats.add_per_image_rmse();
        image_rmse->set_rmse(std::sqrt(residuals.squaredNorm() / 8));
        image_rmse->set_frame_number(frame_num);
        image_rmse->set_camera_name(
            detections_per_view.image().camera_model().frame_name());
      }
      images.push_back(image);
      // cv::imshow("reprojection", image);
      // cv::waitKey(0);
    }
    Image& reprojection_image = *model->add_reprojection_images();
    int image_width = 1280 * 3;
    int image_height = 720 * 2;
    reprojection_image.mutable_camera_model()->set_image_width(image_width);
    reprojection_image.mutable_camera_model()->set_image_height(image_height);
    auto resource_path = GetUniqueArchiveResource(
        FrameNameNumber(
            "reprojection-" + SolverStatus_Name(model->solver_status()),
            frame_num),
        "png", "image/png");
    reprojection_image.mutable_resource()->CopyFrom(resource_path.first);
    LOG(INFO) << resource_path.second.string();
    CHECK(cv::imwrite(
        resource_path.second.string(),
        ConstructGridImage(images, cv::Size(image_width, image_height), 3)))
        << "Could not write: " << resource_path.second;
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

std::vector<MultiViewApriltagDetections> LoadMultiViewApriltagDetections(
    const std::string& root_camera_name, const Resource& event_log,
    const CalibrateMultiViewApriltagRigConfiguration& config) {
  EventLogReader log_reader(event_log);

  std::unordered_set<int> allowed_ids;

  for (auto id : config.tag_ids()) {
    allowed_ids.insert(id);
    LOG(INFO) << "allowed: " << id;
  }

  CHECK(allowed_ids.count(config.root_tag_id()) == 1)
      << "Please ensure root_tag_id is in the tag_ids list";

  std::map<std::string, TimeSeries<Event>> apriltag_series;

  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (std::runtime_error& e) {
      break;
    }
    ApriltagDetections unfiltered_detections;
    if (event.data().UnpackTo(&unfiltered_detections)) {
      ApriltagDetections detections = unfiltered_detections;
      detections.clear_detections();
      for (const auto& detection : unfiltered_detections.detections()) {
        if (allowed_ids.count(detection.id())) {
          detections.add_detections()->CopyFrom(detection);
        }
      }
      event.mutable_data()->PackFrom(detections);

      event.set_name(detections.image().camera_model().frame_name() +
                     "/apriltags");
      apriltag_series[event.name()].insert(event);
    }
  }
  {
    std::stringstream ss;
    ss << "Raw detections\n";
    for (const auto& series : apriltag_series) {
      ss << series.first << " " << series.second.size() << " detections\n";
    }
    LOG(INFO) << ss.str();
  }

  ApriltagsFilter tag_filter;
  std::vector<MultiViewApriltagDetections> mv_detections_series;
  auto time_window =
      google::protobuf::util::TimeUtil::MillisecondsToDuration(1000.0 / 2);

  std::map<std::string, int> detection_counts;

  for (const Event& event : apriltag_series[root_camera_name + "/apriltags"]) {
    ApriltagDetections detections;
    CHECK(event.data().UnpackTo(&detections));
    if (!config.filter_stable_tags() ||
        tag_filter.AddApriltags(detections, 5, 7)) {
      MultiViewApriltagDetections mv_detections;
      for (auto name_series : apriltag_series) {
        auto nearest_event =
            name_series.second.FindNearest(event.stamp(), time_window);
        if (nearest_event) {
          CHECK(nearest_event->data().UnpackTo(
              mv_detections.add_detections_per_view()));
          detection_counts[nearest_event->name()]++;
        }
      }
      mv_detections_series.push_back(mv_detections);
    }
  }
  {
    std::stringstream ss;
    ss << "Stable multi-view detections: " << mv_detections_series.size()
       << "\n";
    for (const auto& name_count : detection_counts) {
      ss << name_count.first << " " << name_count.second
         << " stable detections\n";
    }
    LOG(INFO) << ss.str();
  }

  return mv_detections_series;
}
PoseGraph TagRigFromMultiViewDetections(std::string tag_rig_name,
                                        int root_tag_id,
                                        MultiViewApriltagRigModel* model) {
  model->mutable_apriltag_rig()->set_name(tag_rig_name);
  model->mutable_apriltag_rig()->set_root_tag_id(root_tag_id);

  PoseGraph tag_rig;
  std::map<int, ApriltagRig::Node> tag_rig_nodes;

  for (const auto& mv_detections : model->multi_view_detections()) {
    for (const auto& detections_per_view :
         mv_detections.detections_per_view()) {
      if (detections_per_view.detections_size() <= 1) {
        continue;
      }
      for (const auto& detection : detections_per_view.detections()) {
        if (tag_rig_nodes.count(detection.id())) {
          continue;
        }
        ApriltagRig::Node node;
        node.set_id(detection.id());
        node.set_frame_name(FrameRigTag(tag_rig_name, detection.id()));
        node.set_tag_size(detection.tag_size());
        for (const auto& v : PointsTag(detection)) {
          EigenToProto(v, node.add_points_tag());
        }
        tag_rig_nodes.emplace(detection.id(), std::move(node));
      }
      for (int i = 0; i < detections_per_view.detections_size() - 1; ++i) {
        for (int j = i + 1; j < detections_per_view.detections_size(); ++j) {
          const auto& detection_i = detections_per_view.detections().Get(i);
          const auto& detection_j = detections_per_view.detections().Get(j);
          Sophus::SE3d c_pose_i, c_pose_j;
          CHECK_EQ(detection_i.pose().frame_a(), detection_j.pose().frame_a());
          ProtoToSophus(detection_i.pose().a_pose_b(), &c_pose_i);
          ProtoToSophus(detection_j.pose().a_pose_b(), &c_pose_j);
          tag_rig.AddPose(detection_i.pose().frame_b(),
                          detection_j.pose().frame_b(),
                          c_pose_i.inverse() * c_pose_j);
        }
      }
    }
  }
  std::string root_tag = "tag/" + std::to_string(root_tag_id);

  auto tag_rig_small = tag_rig.AveragePoseGraph(root_tag);

  for (auto& node : tag_rig_nodes) {
    std::string tag = "tag/" + std::to_string(node.first);
    auto root_pose_tag = tag_rig_small.AverageAPoseB(root_tag, tag);
    if (!root_pose_tag) {
      continue;
    }
    SophusToProto(*root_pose_tag, tag_rig_name, node.second.frame_name(),
                  node.second.mutable_pose());
    model->mutable_apriltag_rig()->add_nodes()->CopyFrom(node.second);
  }
  return tag_rig_small;
}

void CameraRigFromMultiViewDetections(
    const CalibrateMultiViewApriltagRigConfiguration& config,
    const PoseGraph& tag_rig, MultiViewApriltagRigModel* model) {
  model->mutable_camera_rig()->set_root_camera_name(config.root_camera_name());
  model->mutable_camera_rig()->set_name(config.name());

  std::string root_tag_name = "tag/" + std::to_string(config.root_tag_id());
  PoseGraph camera_rig_inter;
  PoseGraph camera_rig_tags;

  int frame_num = -1;

  std::map<std::string, CameraModel> camera_models;
  for (const auto& mv_detections : model->multi_view_detections()) {
    frame_num++;
    PoseGraph camera_rig_step_i;

    camera_rig_step_i.AddPose(config.name(), config.root_camera_name(),
                              SE3d::rotX(0));

    for (const auto& detections_per_view :
         mv_detections.detections_per_view()) {
      if (detections_per_view.detections_size() < 1) {
        continue;
      }
      camera_models.emplace(
          detections_per_view.image().camera_model().frame_name(),
          detections_per_view.image().camera_model());

      for (const auto& detection : detections_per_view.detections()) {
        Sophus::SE3d c_pose_tag;
        ProtoToSophus(detection.pose().a_pose_b(), &c_pose_tag);
        auto o_tag_pose_root_tag =
            tag_rig.AverageAPoseB(detection.pose().frame_b(), root_tag_name);
        if (!o_tag_pose_root_tag) {
          LOG(WARNING) << "Unable to compute pose for: "
                       << detection.pose().frame_b() << " <- " << root_tag_name;
          continue;
        }
        auto c_pose_root_tag = c_pose_tag * (*o_tag_pose_root_tag);
        camera_rig_step_i.AddPose(detection.pose().frame_a(), root_tag_name,
                                  c_pose_root_tag);
      }
    }
    if (!camera_rig_step_i.HasName(config.root_camera_name())) {
      continue;
    }
    auto camera_rig_i = camera_rig_step_i.AveragePoseGraph(config.name());

    for (auto es = camera_rig_i.Edges(); es.first != es.second; es.first++) {
      auto edge = camera_rig_i.PoseEdgeMap()[*es.first];
      std::string frame_a = edge.frame_a;
      std::string frame_b = edge.frame_b;
      if (frame_a == root_tag_name || frame_b == root_tag_name) {
        std::string rig_frame_name =
            config.tag_rig_name() + "/view/" + std::to_string(frame_num);
        if (frame_a == root_tag_name) {
          frame_a = rig_frame_name;
        }
        if (frame_b == root_tag_name) {
          frame_b = rig_frame_name;
        }
        camera_rig_tags.AddPose(frame_a, frame_b, edge.GetAPoseB());
      } else {
        camera_rig_inter.AddPose(frame_a, frame_b, edge.GetAPoseB());
      }
    }
  }
  for (auto camera : camera_models) {
    model->mutable_camera_rig()->add_cameras()->CopyFrom(camera.second);
  }

  model->mutable_camera_rig()->mutable_camera_pose_rig()->CopyFrom(
      camera_rig_inter.AveragePoseGraph(config.name()).ToNamedSE3Poses());

  model->mutable_camera_rig_poses_apriltag_rig()->CopyFrom(
      camera_rig_tags.AveragePoseGraph(config.name()).ToNamedSE3Poses());
}

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config) {
  auto dataset_result = ReadProtobufFromResource<CaptureVideoDatasetResult>(
      config.video_dataset());
  MultiViewApriltagRigModel model;

  for (const auto& mv_detections : LoadMultiViewApriltagDetections(
           config.root_camera_name(), dataset_result.dataset(), config)) {
    model.add_multi_view_detections()->CopyFrom(mv_detections);
  }
  auto tag_rig = TagRigFromMultiViewDetections(config.tag_rig_name(),
                                               config.root_tag_id(), &model);
  CameraRigFromMultiViewDetections(config, tag_rig, &model);

  model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);
  ModelError(&model);

  return model;
}

MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    MultiViewApriltagRigModel model) {
  PoseGraph pose_graph = PoseGraphFromModel(model);
  ceres::Problem problem;
  for (PoseEdge* pose_edge : pose_graph.MutablePoseEdges()) {
    LOG(INFO) << *pose_edge;
    problem.AddParameterBlock(pose_edge->GetAPoseB().data(),
                              SE3d::num_parameters,
                              new LocalParameterizationSE3);
  }
  std::string root_tag_frame = FrameRigTag(model.apriltag_rig().name(),
                                           model.apriltag_rig().root_tag_id());
  std::string tag_rig_frame = model.apriltag_rig().name();
  problem.SetParameterBlockConstant(
      pose_graph.MutablePoseEdge(root_tag_frame, tag_rig_frame)
          ->GetAPoseB()
          .data());

  std::string root_camera_frame = model.camera_rig().root_camera_name();
  std::string camera_rig_frame = model.camera_rig().name();

  problem.SetParameterBlockConstant(
      pose_graph.MutablePoseEdge(root_camera_frame, camera_rig_frame)
          ->GetAPoseB()
          .data());

  int frame_num = -1;
  for (const auto& mv_detections : model.multi_view_detections()) {
    frame_num++;

    std::string tag_rig_view_frame =
        tag_rig_frame + "/view/" + std::to_string(frame_num);

    if (!pose_graph.HasEdge(camera_rig_frame, tag_rig_view_frame)) {
      continue;
    }

    PoseEdge* camera_rig_to_tag_rig_view =
        pose_graph.MutablePoseEdge(camera_rig_frame, tag_rig_view_frame);

    for (const auto& detections_per_view :
         mv_detections.detections_per_view()) {
      if (detections_per_view.detections_size() < 1) {
        continue;
      }
      std::string camera_frame =
          detections_per_view.image().camera_model().frame_name();

      PoseEdge* camera_to_camera_rig =
          pose_graph.MutablePoseEdge(camera_frame, camera_rig_frame);

      for (const auto& detection : detections_per_view.detections()) {
        std::string tag_frame = FrameRigTag(tag_rig_frame, detection.id());
        PoseEdge* tag_to_tag_rig =
            pose_graph.MutablePoseEdge(tag_frame, tag_rig_frame);

        ceres::CostFunction* cost_function1 = new ceres::AutoDiffCostFunction<
            CameraRigApriltagRigCostFunctor, 8, Sophus::SE3d::num_parameters,
            Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters>(
            new CameraRigApriltagRigCostFunctor(
                detections_per_view.image().camera_model(),
                PointsTag(detection), PointsImage(detection),
                camera_to_camera_rig->GetAPoseBMap(camera_frame,
                                                   camera_rig_frame),
                tag_to_tag_rig->GetAPoseBMap(tag_rig_frame, tag_frame),
                camera_rig_to_tag_rig_view->GetAPoseBMap(camera_rig_frame,
                                                         tag_rig_view_frame)));
        problem.AddResidualBlock(
            cost_function1, new ceres::CauchyLoss(1.0),
            camera_to_camera_rig->GetAPoseB().data(),
            tag_to_tag_rig->GetAPoseB().data(),
            camera_rig_to_tag_rig_view->GetAPoseB().data());
      }
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
  if (summary.termination_type == ceres::CONVERGENCE) {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_CONVERGED);
  } else {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_FAILED);
  }
  UpdateModelFromPoseGraph(pose_graph, &model);

  ModelError(&model);
  return model;
}

}  // namespace calibration
}  // namespace farm_ng
