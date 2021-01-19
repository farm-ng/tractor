#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ceres/ceres.h>
#include <opencv2/imgproc.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/calibration/camera_rig_apriltag_rig_cost_functor.h"
#include "farm_ng/calibration/camera_rig_apriltag_rig_robot_extrinsics_cost_functor.h"
#include "farm_ng/calibration/local_parameterization.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/image.pb.h"
#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/pose_graph.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"

#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/calibration/capture_robot_extrinsics_dataset.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(dataset, "", "CaptureRobotExtrinsicsResult");
DEFINE_string(initial, "", "Start from an initial model");
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::ReadProtobufFromJsonFile;
using farm_ng::perception::CameraModel;
using farm_ng::perception::Image;
using farm_ng::perception::NamedSE3Pose;
using farm_ng::perception::SE3Map;

typedef farm_ng::core::Event EventPb;

namespace farm_ng::calibration {

RobotExtrinsicsModel RobotExtrinsicsModelFromDatasetResult(
    const CaptureRobotExtrinsicsDatasetResult& dataset_result) {
  perception::ApriltagConfig apriltag_config;
  RobotExtrinsicsModel model;
  model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);

  model.set_workspace_frame_name(
      dataset_result.configuration().workspace_frame_name());
  model.set_base_frame_name(dataset_result.configuration().base_frame_name());
  model.set_link_frame_name(dataset_result.configuration().link_frame_name());

  if (dataset_result.configuration().has_link_tag_rig()) {
    AddApriltagRigToApriltagConfig(
        dataset_result.configuration().link_tag_rig(), &apriltag_config);

    model.mutable_link_tag_rig()->CopyFrom(
        dataset_result.configuration().link_tag_rig());
  }

  if (dataset_result.configuration().has_base_tag_rig()) {
    AddApriltagRigToApriltagConfig(
        dataset_result.configuration().base_tag_rig(), &apriltag_config);

    model.mutable_base_tag_rig()->CopyFrom(
        dataset_result.configuration().base_tag_rig());
  }

  if (dataset_result.configuration().has_base_camera_rig()) {
    model.mutable_base_camera_rig()->CopyFrom(
        dataset_result.configuration().base_camera_rig());
    LOG(INFO)
        << dataset_result.configuration().base_camera_rig().ShortDebugString();
  }

  if (dataset_result.configuration().has_link_camera_rig()) {
    model.mutable_link_camera_rig()->CopyFrom(
        dataset_result.configuration().link_camera_rig());
  }

  std::map<std::string, perception::CameraModel> per_camera_model;
  std::map<std::string, std::unique_ptr<perception::ApriltagDetector>>
      per_camera_detector;

  core::EventLogReader log_reader(dataset_result.dataset());
  perception::ImageLoader image_loader;
  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (const std::runtime_error& e) {
      break;
    }
    CapturePoseResponse pose_response;
    if (event.data().UnpackTo(&pose_response)) {
      RobotExtrinsicsModel::Measurement* measurement = model.add_measurements();

      measurement->mutable_poses()->CopyFrom(pose_response.poses());
      measurement->mutable_joint_states()->CopyFrom(
          pose_response.joint_states());

      for (Image image : pose_response.images()) {
        std::string camera_frame_name = image.camera_model().frame_name();
        if (image.camera_model().image_width() == 1) {
          // TODO remove this hack as after dataset is fixed.
          LOG_FIRST_N(WARNING, 10)
              << "Malformed camera model, defaulting to width=1920 "
                 "height=1080";
          perception::CameraModel* camera_model = image.mutable_camera_model();
          camera_model->set_image_width(1920);
          camera_model->set_image_height(1080);
        }

        if (!per_camera_model.count(camera_frame_name)) {
          per_camera_model[camera_frame_name] = image.camera_model();
          LOG(INFO) << image.camera_model().ShortDebugString();

          per_camera_detector[camera_frame_name].reset(
              new perception::ApriltagDetector(image.camera_model(), nullptr,
                                               &apriltag_config));
        }
        cv::Mat image_mat = image_loader.LoadImage(image);
        cv::Mat depth_mat = image_loader.LoadDepthmap(image);
        cv::Mat gray;
        if (image_mat.channels() == 3) {
          cv::cvtColor(image_mat, gray, cv::COLOR_BGR2GRAY);

        } else {
          CHECK(image_mat.channels() == 1);
          gray = image_mat;
        }
        auto tags = per_camera_detector[camera_frame_name]->Detect(
            gray, depth_mat, event.stamp());
        measurement->mutable_multi_view_detections()
            ->add_detections_per_view()
            ->CopyFrom(tags);
        LOG(INFO) << camera_frame_name << " n tags: " << tags.detections_size();
      }
    }
  }
  return model;
}

void InitializeTagRig(const RobotExtrinsicsModel& model,
                      perception::ApriltagRig* rig) {
  CHECK_GT(rig->nodes_size(), 0);
  CHECK(!rig->name().empty());
  int root_tag_id = rig->root_tag_id();
  if (root_tag_id == 0) {
    root_tag_id = rig->nodes(0).id();
    rig->set_root_tag_id(root_tag_id);
  }

  perception::ApriltagRigIdMap tag_rig_map;
  tag_rig_map.AddRig(*rig);

  perception::PoseGraph pose_graph;
  for (const RobotExtrinsicsModel::Measurement& measurement :
       model.measurements()) {
    for (const perception::ApriltagDetections& detections_per_view :
         measurement.multi_view_detections().detections_per_view()) {
      if (detections_per_view.detections_size() <= 1) {
        continue;
      }
      std::string camera_frame_name =
          detections_per_view.image().camera_model().frame_name();
      for (int i = 0; i < detections_per_view.detections_size() - 1; ++i) {
        for (int j = i + 1; j < detections_per_view.detections_size(); ++j) {
          const auto& detection_i = detections_per_view.detections().Get(i);
          const auto& detection_j = detections_per_view.detections().Get(j);
          auto i_frames = tag_rig_map.GetFrameNames(detection_i.id());
          auto j_frames = tag_rig_map.GetFrameNames(detection_j.id());
          if (!i_frames || !j_frames) {
            continue;
          }
          Sophus::SE3d c_pose_i, c_pose_j;

          CHECK_EQ(camera_frame_name, detection_i.pose().frame_a());
          CHECK_EQ(camera_frame_name, detection_j.pose().frame_a());
          ProtoToSophus(detection_i.pose().a_pose_b(), &c_pose_i);
          ProtoToSophus(detection_j.pose().a_pose_b(), &c_pose_j);

          pose_graph.AddPose(i_frames->tag_frame, j_frames->tag_frame,
                             c_pose_i.inverse() * c_pose_j);
        }
      }
    }
  }

  auto root_frames = tag_rig_map.GetFrameNames(root_tag_id);
  CHECK(root_frames) << "No name for frame id: " << root_tag_id;
  std::string root_tag_frame = root_frames->tag_frame;
  perception::PoseGraph pose_graph_small =
      pose_graph.AveragePoseGraph(root_tag_frame);

  for (perception::ApriltagRig::Node& node : *rig->mutable_nodes()) {
    auto root_pose_tag =
        pose_graph_small.AverageAPoseB(root_tag_frame, node.frame_name());
    if (!root_pose_tag) {
      LOG(WARNING) << "No transform between: " << root_tag_frame << " and "
                   << node.frame_name();
    }
    perception::SophusToProto(*root_pose_tag, rig->name(), node.frame_name(),
                              node.mutable_pose());
    LOG(INFO) << node.ShortDebugString();
  }
}

void InitializeCameraRig(
    const RobotExtrinsicsModel& model, const perception::ApriltagRig& tag_rig,
    perception::MultiViewCameraRig* camera_rig,
    google::protobuf::RepeatedPtrField<farm_ng::perception::NamedSE3Pose>*
        camera_rig_poses_tag_rig) {
  CHECK(!camera_rig->name().empty());
  CHECK_GT(camera_rig->cameras_size(), 0);

  if (camera_rig->root_camera_name().empty()) {
    camera_rig->set_root_camera_name(camera_rig->cameras(0).frame_name());
  }

  CHECK_GT(tag_rig.nodes_size(), 0);
  CHECK(!tag_rig.name().empty());

  int root_tag_id = tag_rig.root_tag_id();

  perception::ApriltagRigIdMap tag_rig_map;
  tag_rig_map.AddRig(tag_rig);
  auto root_tag_frames = tag_rig_map.GetFrameNames(root_tag_id);
  CHECK(root_tag_frames);
  perception::PoseGraph tag_rig_pose_graph;
  for (const perception::ApriltagRig::Node& node : tag_rig.nodes()) {
    tag_rig_pose_graph.AddPose(node.pose());
  }

  perception::PoseGraph camera_rig_inter, camera_rig_tags;
  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);

    // Note that this initialization algorithm assumes that the root camera
    // sees the tag rig, in addition to another camera; such that a relative
    // pose can be computed between the cameras.
    //
    // Given enough measurements this is usually enough to initialize
    // the poses, however is the overlap is very sparse need to revisit -
    // TODO initialize camera rig poses with any pairs of cameras that see the
    // tag rig. Is there a way to express this in the graph?
    perception::PoseGraph camera_rig_step_i;
    camera_rig_step_i.AddPose(camera_rig->name(),
                              camera_rig->root_camera_name(),
                              Sophus::SE3d::rotX(0));
    for (const perception::ApriltagDetections& detections_per_view :
         measurement.multi_view_detections().detections_per_view()) {
      if (detections_per_view.detections_size() <= 1) {
        continue;
      }
      std::string camera_frame_name =
          detections_per_view.image().camera_model().frame_name();
      for (const perception::ApriltagDetection& detection :
           detections_per_view.detections()) {
        auto tag_frames = tag_rig_map.GetFrameNames(detection.id());
        if (!tag_frames) {
          continue;
        }
        std::optional<Sophus::SE3d> tag_pose_rig =
            tag_rig_pose_graph.AverageAPoseB(tag_frames->tag_frame,
                                             tag_rig.name());
        if (!tag_pose_rig) {
          continue;
        }

        CHECK_EQ(camera_frame_name, detection.pose().frame_a());
        Sophus::SE3d camera_pose_tag;
        ProtoToSophus(detection.pose().a_pose_b(), &camera_pose_tag);

        camera_rig_step_i.AddPose(camera_frame_name, tag_rig.name(),
                                  camera_pose_tag * (*tag_pose_rig));
      }
    }
    auto camera_rig_i = camera_rig_step_i.AveragePoseGraph(camera_rig->name());
    std::stringstream ss;
    for (auto es = camera_rig_i.Edges(); es.first != es.second; es.first++) {
      auto edge = camera_rig_i.PoseEdgeMap()[*es.first];
      std::string frame_a = edge.frame_a;
      std::string frame_b = edge.frame_b;

      if (frame_a == tag_rig.name() || frame_b == tag_rig.name()) {
        std::string rig_frame_name =
            tag_rig.name() + "/view/" + std::to_string(frame_num);
        if (frame_a == tag_rig.name()) {
          frame_a = rig_frame_name;
        }
        if (frame_b == tag_rig.name()) {
          frame_b = rig_frame_name;
        }
        camera_rig_tags.AddPose(frame_a, frame_b, edge.GetAPoseB());
      } else {
        camera_rig_inter.AddPose(frame_a, frame_b, edge.GetAPoseB());
      }
      ss << frame_a << " -> " << frame_b << ", ";
    }
    LOG(INFO) << "View " << frame_num << " initialized with " << ss.str();
  }

  for (const auto& camera : camera_rig->cameras()) {
    if (!camera_rig_inter.HasEdge(camera_rig->name(), camera.frame_name())) {
      LOG(WARNING)
          << camera.frame_name()
          << " is not initialized, no views with overlap with root camera?";
    } else {
      const auto& edge =
          camera_rig_inter.GetPoseEdge(camera_rig->name(), camera.frame_name());
      LOG(INFO) << edge.frame_a << " -> " << edge.frame_b
                << " n views: " << edge.a_poses_b.size();
    }
  }

  camera_rig_inter = camera_rig_inter.AveragePoseGraph(camera_rig->name());
  camera_rig->mutable_camera_pose_rig()->CopyFrom(
      camera_rig_inter.AveragePoseGraph(camera_rig->name()).ToNamedSE3Poses());
  camera_rig_poses_tag_rig->CopyFrom(
      camera_rig_tags.AveragePoseGraph(camera_rig->name()).ToNamedSE3Poses());
}

perception::PoseGraph PoseGraphFromModel(const RobotExtrinsicsModel& model) {
  perception::PoseGraph pose_graph;
  if (model.has_link_tag_rig()) {
    for (const perception::ApriltagRig::Node& node :
         model.link_tag_rig().nodes()) {
      pose_graph.AddPose(node.pose());
    }
  }
  if (model.has_base_tag_rig()) {
    for (const perception::ApriltagRig::Node& node :
         model.base_tag_rig().nodes()) {
      pose_graph.AddPose(node.pose());
    }
  }
  if (model.has_link_camera_rig()) {
    for (const NamedSE3Pose& camera_pose_rig :
         model.link_camera_rig().camera_pose_rig()) {
      pose_graph.AddPose(camera_pose_rig);
    }
    for (const NamedSE3Pose& camera_rig_pose_tag_rig :
         model.link_camera_rig_poses_base_tag_rig()) {
      CHECK_EQ(camera_rig_pose_tag_rig.frame_a(),
               model.link_camera_rig().name());
      CHECK(camera_rig_pose_tag_rig.frame_b().rfind(
                model.base_tag_rig().name()) == 0)
          << camera_rig_pose_tag_rig.frame_b()
          << " doesn't start with: " << model.base_tag_rig().name();
      pose_graph.AddPose(camera_rig_pose_tag_rig);
    }
  }
  if (model.has_base_camera_rig()) {
    for (const NamedSE3Pose& camera_pose_rig :
         model.base_camera_rig().camera_pose_rig()) {
      pose_graph.AddPose(camera_pose_rig);
    }
    for (const NamedSE3Pose& camera_rig_pose_tag_rig :
         model.base_camera_rig_poses_link_tag_rig()) {
      CHECK_EQ(camera_rig_pose_tag_rig.frame_a(),
               model.base_camera_rig().name());
      CHECK(camera_rig_pose_tag_rig.frame_b().rfind(
                model.link_tag_rig().name()) == 0)
          << camera_rig_pose_tag_rig.frame_b()
          << " doesn't start with: " << model.link_tag_rig().name();

      pose_graph.AddPose(camera_rig_pose_tag_rig);
    }
  }
  pose_graph.AddPose(model.base_frame_name(), model.base_camera_rig().name(),
                     Sophus::SE3d::rotX(0));
  pose_graph.AddPose(model.link_frame_name(), model.link_tag_rig().name(),
                     Sophus::SE3d::rotX(0));

  return pose_graph;
}

RobotExtrinsicsModel SolveRobotExtrinsicsModel(RobotExtrinsicsModel model) {
  if (model.has_link_tag_rig()) {
    InitializeTagRig(model, model.mutable_link_tag_rig());
    InitializeCameraRig(model, model.link_tag_rig(),
                        model.mutable_base_camera_rig(),
                        model.mutable_base_camera_rig_poses_link_tag_rig());
  }
  if (model.has_base_tag_rig()) {
    InitializeTagRig(model, model.mutable_base_tag_rig());
    InitializeCameraRig(model, model.base_tag_rig(),
                        model.mutable_link_camera_rig(),
                        model.mutable_link_camera_rig_poses_base_tag_rig());
  }

  perception::PoseGraph pose_graph = PoseGraphFromModel(model);
  ceres::Problem problem;
  for (perception::PoseEdge* pose_edge : pose_graph.MutablePoseEdges()) {
    LOG(INFO) << *pose_edge;
    problem.AddParameterBlock(pose_edge->GetAPoseB().data(),
                              Sophus::SE3d::num_parameters,
                              new LocalParameterizationSE3);
  }

  problem.SetParameterBlockConstant(
      pose_graph
          .MutablePoseEdge(model.base_camera_rig().name(),
                           model.base_camera_rig().root_camera_name())
          ->GetAPoseB()
          .data());

  problem.SetParameterBlockConstant(
      pose_graph
          .MutablePoseEdge(
              model.link_tag_rig().name(),
              perception::FrameRigTag(model.link_tag_rig().name(),
                                      model.link_tag_rig().root_tag_id()))
          ->GetAPoseB()
          .data());

  std::string camera_rig_frame = model.base_camera_rig().name();
  std::string tag_rig_frame = model.link_tag_rig().name();

  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);

    std::string tag_rig_view_frame =
        tag_rig_frame + "/view/" + std::to_string(frame_num);

    if (!pose_graph.HasEdge(camera_rig_frame, tag_rig_view_frame)) {
      continue;
    }

    perception::PoseEdge* camera_rig_to_tag_rig_view =
        pose_graph.MutablePoseEdge(camera_rig_frame, tag_rig_view_frame);

    for (const auto& detections_per_view :
         measurement.multi_view_detections().detections_per_view()) {
      if (detections_per_view.detections_size() < 1) {
        continue;
      }
      std::string camera_frame =
          detections_per_view.image().camera_model().frame_name();

      perception::PoseEdge* camera_to_camera_rig =
          pose_graph.MutablePoseEdge(camera_frame, camera_rig_frame);

      std::optional<Sophus::SE3d> base_pose_link;

      for (const perception::NamedSE3Pose& pose : measurement.poses()) {
        if ((pose.frame_a() == model.base_frame_name() &&
             pose.frame_b() == model.link_frame_name())) {
          CHECK(!base_pose_link);
          Sophus::SE3d a_pose_b;
          perception::ProtoToSophus(pose.a_pose_b(), &a_pose_b);
          base_pose_link = a_pose_b;
        }
      }
      CHECK(base_pose_link)
          << "No base pose link set: " << measurement.ShortDebugString();

      for (const auto& detection : detections_per_view.detections()) {
        std::string tag_frame =
            perception::FrameRigTag(tag_rig_frame, detection.id());
        perception::PoseEdge* tag_to_tag_rig =
            pose_graph.MutablePoseEdge(tag_frame, tag_rig_frame);
        perception::PoseEdge* base_to_camera_rig = pose_graph.MutablePoseEdge(
            model.base_frame_name(), camera_rig_frame);

        perception::PoseEdge* link_to_tag_rig =
            pose_graph.MutablePoseEdge(model.link_frame_name(), tag_rig_frame);
        ceres::CostFunction* cost_function1 = new ceres::AutoDiffCostFunction<
            CameraRigApriltagRigRobotExtrinsicsCostFunctor, 6,
            Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters,
            Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters,
            Sophus::SE3d::num_parameters>(
            new CameraRigApriltagRigRobotExtrinsicsCostFunctor(
                detections_per_view.image().camera_model(),
                PointsTag(detection), PointsImage(detection), *base_pose_link,
                camera_to_camera_rig->GetAPoseBMap(camera_frame,
                                                   camera_rig_frame),
                tag_to_tag_rig->GetAPoseBMap(tag_rig_frame, tag_frame),
                camera_rig_to_tag_rig_view->GetAPoseBMap(camera_rig_frame,
                                                         tag_rig_view_frame),
                base_to_camera_rig->GetAPoseBMap(model.base_frame_name(),
                                                 camera_rig_frame),
                link_to_tag_rig->GetAPoseBMap(model.link_frame_name(),
                                              tag_rig_frame)

                    ));
        problem.AddResidualBlock(cost_function1, nullptr,
                                 camera_to_camera_rig->GetAPoseB().data(),
                                 tag_to_tag_rig->GetAPoseB().data(),
                                 camera_rig_to_tag_rig_view->GetAPoseB().data(),
                                 base_to_camera_rig->GetAPoseB().data(),
                                 link_to_tag_rig->GetAPoseB().data());

        ceres::CostFunction* cost_function2 = new ceres::AutoDiffCostFunction<
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
            cost_function2, new ceres::HuberLoss(2.0),
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
  options.minimizer_progress_to_stdout = false;
  ceres::Solve(options, &problem, &summary);
  LOG(INFO) << summary.FullReport() << std::endl;
  if (summary.termination_type == ceres::CONVERGENCE) {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_CONVERGED);
  } else {
    model.set_solver_status(SolverStatus::SOLVER_STATUS_FAILED);
  }

  for (perception::PoseEdge* pose_edge : pose_graph.MutablePoseEdges()) {
    LOG(INFO) << *pose_edge;
  }
  return model;
}

class CalibrateRobotExtrinsicsProgram {
 public:
  CalibrateRobotExtrinsicsProgram(core::EventBus& bus, bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      // status_.mutable_input_required_resource()->CopyFrom(resource);
    } else {
      // set_configuration(resource);
    }
    bus_.AddSubscriptions({bus_.GetName()});
    bus_.GetEventSignal()->connect(
        std::bind(&CalibrateRobotExtrinsicsProgram::on_event, this,
                  std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  void send_status() {
    // bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&CalibrateRobotExtrinsicsProgram::on_timer,
                                this, std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    core::Resource configuration_resource;
    if (!event.data().UnpackTo(&configuration_resource)) {
      return false;
    }
    LOG(INFO) << configuration_resource.ShortDebugString();
    set_configuration(configuration_resource);
    return true;
  }

  void set_configuration(const core::Resource& resource) { send_status(); }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

  int run() {
    // while (status_.has_input_required_resource()) {
    //      bus_.get_io_service().run_one();
    //}

    WaitForServices(bus_, {});

    core::Resource dataset_resource;
    dataset_resource.set_path(FLAGS_dataset);
    dataset_resource.set_content_type(
        core::ContentTypeProtobufJson<CaptureRobotExtrinsicsDatasetResult>());
    CaptureRobotExtrinsicsDatasetResult dataset_result =
        core::ReadProtobufFromResource<CaptureRobotExtrinsicsDatasetResult>(
            dataset_resource);
    auto output_dir =
        boost::filesystem::path(dataset_result.dataset().path()).parent_path();

    // Output under the same directory as the dataset.
    core::SetArchivePath((output_dir / "robot_extrinsics").string());

    RobotExtrinsicsModel model;
    core::Resource initial_resource;

    if (FLAGS_initial.empty()) {
      model = RobotExtrinsicsModelFromDatasetResult(dataset_result);

      initial_resource =
          core::ArchiveProtobufAsBinaryResource("initial", model);
      LOG(INFO) << "Wrote initial model to blobstore: "
                << initial_resource.path();
    } else {
      initial_resource.set_path(FLAGS_initial);
      initial_resource.set_content_type(
          core::ContentTypeProtobufBinary<RobotExtrinsicsModel>());
      model = core::ReadProtobufFromResource<RobotExtrinsicsModel>(
          initial_resource);
      // LOG(INFO) << model.ShortDebugString();
    }

    SolveRobotExtrinsicsModel(model);

    return 0;
  }

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;
};

}  // namespace farm_ng::calibration

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::CalibrateRobotExtrinsicsProgram program(
      bus, FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup"; }

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
