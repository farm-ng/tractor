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
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/image.pb.h"
#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/pose_graph.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/robot_arm_fk.h"
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
using farm_ng::perception::RobotArmFK6dof;
using farm_ng::perception::RobotLinkFK;
using farm_ng::perception::SE3Map;

typedef farm_ng::core::Event EventPb;

namespace farm_ng::calibration {

RobotArmExtrinsicsModel RobotArmExtrinsicsModelFromDatasetResult(
    const CaptureRobotExtrinsicsDatasetResult& dataset_result) {
  perception::ApriltagConfig apriltag_config;
  RobotArmExtrinsicsModel model;
  model.mutable_robot_arm()->CopyFrom(
      dataset_result.configuration().robot_arm());
  RobotArmFK6dof(dataset_result.configuration().robot_arm());
  model.set_base_frame_name(dataset_result.configuration().base_frame_name());
  model.set_link_frame_name(dataset_result.configuration().link_frame_name());

  if (dataset_result.configuration().has_link_tag_rig()) {
    AddApriltagRigToApriltagConfig(
        dataset_result.configuration().link_tag_rig(), &apriltag_config);

    model.mutable_base_camera_rig_model()->mutable_apriltag_rig()->CopyFrom(
        dataset_result.configuration().link_tag_rig());
  }

  if (dataset_result.configuration().has_base_tag_rig()) {
    LOG(WARNING) << "base_tag_rig not supported yet.";
  }

  if (dataset_result.configuration().has_base_camera_rig()) {
    model.mutable_base_camera_rig_model()->mutable_camera_rig()->CopyFrom(
        dataset_result.configuration().base_camera_rig());
    LOG(INFO)
        << dataset_result.configuration().base_camera_rig().ShortDebugString();
  }

  if (dataset_result.configuration().has_link_camera_rig()) {
    LOG(WARNING) << "link_camera_rig not supported yet.";
  }

  std::map<std::string, perception::CameraModel> per_camera_model;
  std::map<std::string, std::unique_ptr<perception::ApriltagDetector>>
      per_camera_detector;

  core::EventLogReader log_reader(dataset_result.dataset());
  perception::ImageLoader image_loader;
  CapturePoseRequest pose_req;

  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (const std::runtime_error& e) {
      break;
    }

    if (event.data().UnpackTo(&pose_req)) {
      LOG(INFO) << "Request:\n" << pose_req.ShortDebugString();
    }
    CapturePoseResponse pose_response;
    if (event.data().UnpackTo(&pose_response)) {
      LOG(INFO) << "Response:\n" << pose_response.ShortDebugString();
      RobotArmExtrinsicsModel::Measurement* measurement =
          model.add_measurements();
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
        tags.mutable_image()->CopyFrom(image);
        measurement->mutable_multi_view_detections()
            ->add_detections_per_view()
            ->CopyFrom(tags);
        LOG(INFO) << camera_frame_name << " n tags: " << tags.detections_size();
      }
    }
  }
  return model;
}

struct PoseCostFunction {
  PoseCostFunction(Sophus::SE3d a_pose_b, Sophus::SE3d c_pose_d)
      : a_pose_b_(a_pose_b), c_pose_d_(c_pose_d) {}

  template <class T>
  bool operator()(T const* const raw_b_pose_c, T const* const raw_a_pose_d,
                  T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_0(raw_residuals);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_1(raw_residuals + 3);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_2(raw_residuals + 6);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_3(raw_residuals + 9);

    Eigen::Map<Sophus::SE3<T> const> b_pose_c(raw_b_pose_c);
    Eigen::Map<Sophus::SE3<T> const> a_pose_d(raw_a_pose_d);
    auto d_pose_c = c_pose_d_.inverse().cast<T>();
    auto c_pose_b = b_pose_c.inverse();
    auto b_pose_a = a_pose_b_.inverse().cast<T>();
    Sophus::SE3<T> a_pose_a = a_pose_d * d_pose_c * c_pose_b * b_pose_a;
    typedef Eigen::Matrix<T, 3, 1> Point3;
    Point3 orig(T(0.0), T(0.0), T(0.0));
    Point3 x(T(1.0), T(0.0), T(0.0));
    Point3 y(T(0.0), T(1.0), T(0.0));
    Point3 z(T(0.0), T(0.0), T(1.0));
    residuals_0 = z - (a_pose_a * z);
    residuals_1 = x - (a_pose_a * x);
    residuals_2 = y - (a_pose_a * y);
    residuals_3 = a_pose_a * orig;

    return true;
  }

  Sophus::SE3d a_pose_b_;
  Sophus::SE3d c_pose_d_;
};

void DualPoseEstimate(const std::vector<Sophus::SE3d>& a_poses_b,
                      const std::vector<Sophus::SE3d>& c_poses_d,
                      Sophus::SE3d* b_pose_c, Sophus::SE3d* a_pose_d) {
  CHECK_EQ(a_poses_b.size(), c_poses_d.size());
  CHECK_GT(a_poses_b.size(), 1);
  ceres::Problem problem;

  problem.AddParameterBlock(b_pose_c->data(), Sophus::SE3d::num_parameters,
                            new LocalParameterizationSE3);

  problem.AddParameterBlock(a_pose_d->data(), Sophus::SE3d::num_parameters,
                            new LocalParameterizationSE3);

  for (size_t i = 0; i < a_poses_b.size(); ++i) {
    Sophus::SE3d a_i_pose_b_i = a_poses_b[i];
    Sophus::SE3d c_i_pose_d_i = c_poses_d[i];

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PoseCostFunction, 12,
                                        Sophus::SE3d::num_parameters,
                                        Sophus::SE3d::num_parameters>(
            new PoseCostFunction(a_poses_b[i], c_poses_d[i]));

    problem.AddResidualBlock(cost_function, nullptr, b_pose_c->data(),
                             a_pose_d->data());
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
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
}
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Sophus::SE3d;

RobotArmExtrinsicsModel SolveRobotArmExtrinsicsModel(
    RobotArmExtrinsicsModel model) {
  CalibrateMultiViewApriltagRigResult result;

  result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

  MultiViewApriltagRigModel camera_tag_model;
  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotArmExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);
    camera_tag_model.add_multi_view_detections()->CopyFrom(
        measurement.multi_view_detections());
  }

  auto camera_rig = model.base_camera_rig_model().camera_rig();
  auto apriltag_rig = model.base_camera_rig_model().apriltag_rig();

  auto tag_posegraph = TagRigFromMultiViewDetections(
      apriltag_rig.name(), apriltag_rig.root_tag_id(), &camera_tag_model);

  CameraRigFromMultiViewDetections(camera_rig.name(),
                                   camera_rig.root_camera_name(), tag_posegraph,
                                   &camera_tag_model);
  camera_tag_model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);
  camera_tag_model = SolveMultiViewApriltagModel(camera_tag_model);

  camera_tag_model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);
  // ModelError(&camera_tag_model);

  result.mutable_multi_view_apriltag_rig_solved()->CopyFrom(
      core::ArchiveProtobufAsBinaryResource("initial", camera_tag_model));
  result.set_rmse(camera_tag_model.rmse());
  result.set_solver_status(camera_tag_model.solver_status());
  result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
  core::ArchiveProtobufAsJsonResource(camera_rig.name(), result);
  core::WriteProtobufAsJsonResource(core::BUCKET_APRILTAG_RIG_MODELS,
                                    camera_rig.name(), result);
  model.mutable_base_camera_rig_model()->CopyFrom(camera_tag_model);

  std::vector<Sophus::SE3d> camera_rig_poses_tag_rig, link_poses_base;
  perception::PoseGraph pose_graph;
  std::string camera_rig_name = camera_rig.name();
  for (const NamedSE3Pose& camera_rig_pose_apriltag_rig :
       camera_tag_model.camera_rig_poses_apriltag_rig()) {
    pose_graph.AddPose(camera_rig_pose_apriltag_rig);
  }
  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotArmExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);

    std::string rig_frame_name = camera_tag_model.apriltag_rig().name() +
                                 "/view/" + std::to_string(frame_num);
    if (!pose_graph.HasEdge(camera_rig_name, rig_frame_name)) {
      LOG(INFO) << "no edge: " << camera_rig_name << " " << rig_frame_name;
      continue;
    }

    auto camera_pose_tag =
        pose_graph.AverageAPoseB(camera_rig_name, rig_frame_name);
    CHECK(camera_pose_tag);

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
    CHECK(base_pose_link) << "No base pose link set: "
                          << measurement.ShortDebugString();

    VLOG(3) << "camera_pose_tag: t: "
            << camera_pose_tag->translation().transpose()
            << " r: " << camera_pose_tag->unit_quaternion().vec().transpose();
    LOG(INFO) << "base_pose_link: t: "
              << base_pose_link->translation().transpose()
              << " r: " << base_pose_link->unit_quaternion().x() << " "
              << base_pose_link->unit_quaternion().y() << " "
              << base_pose_link->unit_quaternion().z() << " "
              << base_pose_link->unit_quaternion().w();

    camera_rig_poses_tag_rig.push_back(*camera_pose_tag);
    link_poses_base.push_back(base_pose_link->inverse());
  }
  Sophus::SE3d tag_rig_pose_link = Sophus::SE3d::rotX(0);
  Sophus::SE3d camera_rig_pose_base = Sophus::SE3d::rotY(0);

  DualPoseEstimate(camera_rig_poses_tag_rig, link_poses_base,
                   &tag_rig_pose_link, &camera_rig_pose_base);

  SophusToProto(tag_rig_pose_link, camera_tag_model.apriltag_rig().name(),
                model.link_frame_name(), model.add_workspace_poses());

  SophusToProto(camera_rig_pose_base, camera_tag_model.camera_rig().name(),
                model.base_frame_name(), model.add_workspace_poses());

  perception::PoseGraph camera_pg;
  camera_pg.AddPose(camera_tag_model.camera_rig().name(),
                    model.base_frame_name(), camera_rig_pose_base);
  camera_pg.AddPoses(camera_tag_model.camera_rig().camera_pose_rig());

  camera_pg = camera_pg.AveragePoseGraph(model.base_frame_name());
  std::stringstream ss;
  for (auto pose : camera_pg.ToNamedSE3Poses()) {
    ss << pose.ShortDebugString() << std::endl;
  }
  LOG(INFO) << "Camera pose graph:\n" << ss.str();
  return model;
}

struct CameraRigApriltagRig6dofRobotExtrinsicsCostFunctor {
  CameraRigApriltagRig6dofRobotExtrinsicsCostFunctor(
      const CameraModel& camera, const perception::ApriltagDetection& detection,
      perception::RobotArmFK6dof fk, Eigen::Matrix<double, 6, 1> joint_values,
      SE3Map camera_pose_camera_rig, SE3Map tag_rig_pose_tag,
      SE3Map base_pose_camera_rig, SE3Map link_pose_tag_rig,
      double depth_scale = 100.0)
      : camera_(camera),
        points_tag_(PointsTag(detection)),
        points_image_(PointsImage(detection)),
        depths_camera_({0.0}),
        fk_(fk),
        joint_values_(joint_values),
        camera_pose_camera_rig_(camera_pose_camera_rig),
        tag_rig_pose_tag_(tag_rig_pose_tag),
        base_pose_camera_rig_(base_pose_camera_rig),
        link_pose_tag_rig_(link_pose_tag_rig),
        depth_scale_(depth_scale) {
    CHECK(detection.depth_size() == 0 || detection.depth_size() == 4)
        << detection.ShortDebugString();
    if (detection.depth_size() == 4) {
      for (int i = 0; i < detection.depth_size(); ++i) {
        if (detection.depth(i).IsInitialized()) {
          depths_camera_[i] = detection.depth(i).value();
        }
      }
    }
  }

  template <class T>
  Eigen::Matrix<T, 4, 3> TransformToCamera(
      T const* const raw_camera_pose_camera_rig,
      T const* const raw_tag_rig_pose_tag,
      T const* const raw_base_pose_camera_rig,
      T const* const raw_link_pose_tag_rig,
      T const* const raw_joint_offsets) const {
    Eigen::Map<const Eigen::Matrix<T, 6, 1>> joint_offsets(raw_joint_offsets);
    auto camera_pose_base =
        camera_pose_camera_rig_.Map(raw_camera_pose_camera_rig) *
        base_pose_camera_rig_.Map(raw_base_pose_camera_rig).inverse();

    auto link_pose_tag = link_pose_tag_rig_.Map(raw_link_pose_tag_rig) *
                         tag_rig_pose_tag_.Map(raw_tag_rig_pose_tag);

    auto base_pose_link = fk_.FK<T>(joint_values_.cast<T>(), joint_offsets);
    auto camera_pose_tag = camera_pose_base * base_pose_link * link_pose_tag;
    Eigen::Matrix<T, 4, 3> points_camera;
    for (int i = 0; i < 4; ++i) {
      points_camera.row(i) = camera_pose_tag * points_tag_[i].cast<T>();
    }
    return points_camera;
  }

  template <class T>
  bool operator()(T const* const raw_camera_pose_camera_rig,
                  T const* const raw_tag_rig_pose_tag,
                  T const* const raw_base_pose_camera_rig,
                  T const* const raw_link_pose_tag_rig,
                  T const* const raw_joint_offsets, T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 4, 3>> residuals(raw_residuals);
    Eigen::Matrix<T, 4, 3> points_camera = TransformToCamera(
        raw_camera_pose_camera_rig, raw_tag_rig_pose_tag,
        raw_base_pose_camera_rig, raw_link_pose_tag_rig, raw_joint_offsets);

    for (int i = 0; i < 4; ++i) {
      residuals.row(i).template head<2>() =
          points_image_[i].cast<T>() -
          perception::ProjectPointToPixel<T>(camera_, points_camera.row(i));
      if (depths_camera_[i] > 0.0 && points_camera(i, 2) > T(0.0)) {
        residuals(i, 2) =
            T(depth_scale_) * (points_camera(i, 2) - T(depths_camera_[i]));
      } else {
        residuals(i, 2) = T(0.0);
      }
    }

    return true;
  }
  CameraModel camera_;
  std::array<Eigen::Vector3d, 4> points_tag_;
  std::array<Eigen::Vector2d, 4> points_image_;
  std::array<double, 4> depths_camera_;
  perception::RobotArmFK6dof fk_;
  Eigen::Matrix<double, 6, 1> joint_values_;

  SE3Map camera_pose_camera_rig_;
  SE3Map tag_rig_pose_tag_;
  SE3Map base_pose_camera_rig_;
  SE3Map link_pose_tag_rig_;

  double depth_scale_;
};

RobotArmExtrinsicsModel SolveRobotArmExtrinsicsModel2(
    RobotArmExtrinsicsModel model) {
  RobotArmFK6dof fk(model.robot_arm());
  perception::PoseGraph pose_graph;
  pose_graph.AddPoses(model.workspace_poses());
  for (const perception::ApriltagRig::Node& node :
       model.base_camera_rig_model().apriltag_rig().nodes()) {
    pose_graph.AddPose(node.pose());
  }
  pose_graph.AddPoses(
      model.base_camera_rig_model().camera_rig().camera_pose_rig());

  ceres::Problem problem;
  for (perception::PoseEdge* pose_edge : pose_graph.MutablePoseEdges()) {
    LOG(INFO) << *pose_edge;
    problem.AddParameterBlock(pose_edge->GetAPoseB().data(),
                              Sophus::SE3d::num_parameters,
                              new LocalParameterizationSE3);
    for (auto ws_pose : model.workspace_poses()) {
      if ((pose_edge->frame_a == ws_pose.frame_a() &&
           pose_edge->frame_b == ws_pose.frame_b()) ||
          (pose_edge->frame_a == ws_pose.frame_b() &&
           pose_edge->frame_b == ws_pose.frame_a())) {
        // problem.SetParameterBlockVariable(pose_edge->GetAPoseB().data());
      } else {
        // problem.SetParameterBlockConstant(pose_edge->GetAPoseB().data());
      }
    }
  }

  std::string camera_rig_frame =
      model.base_camera_rig_model().camera_rig().name();
  std::string root_camera_name =
      model.base_camera_rig_model().camera_rig().root_camera_name();
  std::string tag_rig_frame =
      model.base_camera_rig_model().apriltag_rig().name();
  std::string root_tag_name = perception::FrameRigTag(
      tag_rig_frame,
      model.base_camera_rig_model().apriltag_rig().root_tag_id());

  problem.SetParameterBlockConstant(
      pose_graph.MutablePoseEdge(camera_rig_frame, root_camera_name)
          ->GetAPoseB()
          .data());

  problem.SetParameterBlockConstant(
      pose_graph.MutablePoseEdge(tag_rig_frame, root_tag_name)
          ->GetAPoseB()
          .data());

  std::map<int, Sophus::SE3d> base_poses_link;
  std::map<int, Eigen::Matrix<double, 6, 1>> joint_values_per_frame;
  Eigen::Matrix<double, 6, 1> joint_offsets =
      Eigen::Matrix<double, 6, 1>::Zero();

  problem.AddParameterBlock(
      joint_offsets.data(), joint_offsets.rows(),
      new ceres::SubsetParameterization(6, std::vector<int32_t>({0, 5})));

  // problem.SetParameterBlockConstant(joint_offsets.data());
  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotArmExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);
    Eigen::Matrix<double, 6, 1> joint_values;
    CHECK(measurement.joint_states_size() == 6);
    for (int i = 0; i < measurement.joint_states_size(); ++i) {
      CHECK_EQ(measurement.joint_states(i).units(),
               perception::JointState::UNITS_RADIANS);
      joint_values(i) = measurement.joint_states(i).value();
    }
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
    CHECK(base_pose_link) << "No base pose link set: "
                          << measurement.ShortDebugString();
    base_poses_link[frame_num] = *base_pose_link;
    joint_values_per_frame[frame_num] = joint_values;
    SE3d base_pose_link_fk = fk.FK(joint_values, joint_offsets);

    LOG(INFO) << "base_pose_link*base_pose_link_fk.inverse()\n"
              << (*base_pose_link * base_pose_link_fk.inverse()).matrix();

    for (const auto& detections_per_view :
         measurement.multi_view_detections().detections_per_view()) {
      if (detections_per_view.detections_size() < 1) {
        continue;
      }
      std::string camera_frame =
          detections_per_view.image().camera_model().frame_name();

      perception::PoseEdge* camera_to_camera_rig =
          pose_graph.MutablePoseEdge(camera_frame, camera_rig_frame);

      perception::PoseEdge* base_to_camera_rig =
          pose_graph.MutablePoseEdge(model.base_frame_name(), camera_rig_frame);

      perception::PoseEdge* link_to_tag_rig =
          pose_graph.MutablePoseEdge(model.link_frame_name(), tag_rig_frame);

      for (const auto& detection : detections_per_view.detections()) {
        std::string tag_frame =
            perception::FrameRigTag(tag_rig_frame, detection.id());
        perception::PoseEdge* tag_to_tag_rig =
            pose_graph.MutablePoseEdge(tag_frame, tag_rig_frame);

        ceres::CostFunction* cost_function1 = new ceres::AutoDiffCostFunction<
            CameraRigApriltagRig6dofRobotExtrinsicsCostFunctor, 12,
            Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters,
            Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters, 6>(
            new CameraRigApriltagRig6dofRobotExtrinsicsCostFunctor(
                detections_per_view.image().camera_model(), detection, fk,
                joint_values,
                camera_to_camera_rig->GetAPoseBMap(camera_frame,
                                                   camera_rig_frame),
                tag_to_tag_rig->GetAPoseBMap(tag_rig_frame, tag_frame),
                base_to_camera_rig->GetAPoseBMap(model.base_frame_name(),
                                                 camera_rig_frame),
                link_to_tag_rig->GetAPoseBMap(model.link_frame_name(),
                                              tag_rig_frame),
                100.0));

        problem.AddResidualBlock(
            cost_function1, nullptr,  // new ceres::CauchyLoss(1.0),
            camera_to_camera_rig->GetAPoseB().data(),
            tag_to_tag_rig->GetAPoseB().data(),
            base_to_camera_rig->GetAPoseB().data(),
            link_to_tag_rig->GetAPoseB().data(), joint_offsets.data());
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
    model.mutable_base_camera_rig_model()->set_solver_status(
        SolverStatus::SOLVER_STATUS_CONVERGED);
  } else {
    model.mutable_base_camera_rig_model()->set_solver_status(
        SolverStatus::SOLVER_STATUS_FAILED);
  }
  LOG(INFO) << "Joint offsets (degrees): "
            << (180.0 / M_PI) * joint_offsets.transpose();
  pose_graph.UpdateNamedSE3Poses(model.mutable_workspace_poses());
  auto* rig_model = model.mutable_base_camera_rig_model();
  for (perception::ApriltagRig::Node& node :
       *rig_model->mutable_apriltag_rig()->mutable_nodes()) {
    pose_graph.UpdateNamedSE3Pose(node.mutable_pose());
  }
  pose_graph.UpdateNamedSE3Poses(
      rig_model->mutable_camera_rig()->mutable_camera_pose_rig());

  perception::PoseGraph camera_tag_posegraph;
  camera_tag_posegraph.AddPoses(rig_model->camera_rig_poses_apriltag_rig());

  auto camera_rig_pose_base =
      pose_graph.AverageAPoseB(camera_rig_frame, model.base_frame_name());
  CHECK(camera_rig_pose_base);

  auto link_pose_tag_rig =
      pose_graph.AverageAPoseB(model.link_frame_name(), tag_rig_frame);
  CHECK(link_pose_tag_rig);
  rig_model->clear_camera_rig_poses_apriltag_rig();
  for (auto frame_num_base_pose_link : base_poses_link) {
    auto joint_values = joint_values_per_frame[frame_num_base_pose_link.first];
    SE3d base_pose_link = fk.FK(joint_values, joint_offsets);
    Sophus::SE3d camera_rig_pose_apriltag_rig =
        (*camera_rig_pose_base) * base_pose_link * (*link_pose_tag_rig);

    perception::SophusToProto(
        camera_rig_pose_apriltag_rig, camera_rig_frame,
        tag_rig_frame + "/view/" +
            std::to_string(frame_num_base_pose_link.first),
        rig_model->add_camera_rig_poses_apriltag_rig());
  }

  ModelError(rig_model);
  for (auto pose :
       pose_graph.AveragePoseGraph(model.base_frame_name()).ToNamedSE3Poses()) {
    LOG(INFO) << pose.ShortDebugString();
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

    RobotArmExtrinsicsModel model;
    core::Resource initial_resource;

    if (FLAGS_initial.empty()) {
      model = RobotArmExtrinsicsModelFromDatasetResult(dataset_result);

      initial_resource =
          core::ArchiveProtobufAsBinaryResource("initial", model);
      LOG(INFO) << "Wrote initial model to blobstore: "
                << initial_resource.path();
    } else {
      initial_resource.set_path(FLAGS_initial);
      initial_resource.set_content_type(
          core::ContentTypeProtobufBinary<RobotArmExtrinsicsModel>());
      model = core::ReadProtobufFromResource<RobotArmExtrinsicsModel>(
          initial_resource);
      // LOG(INFO) << model.ShortDebugString();
    }

    model = SolveRobotArmExtrinsicsModel(model);
    SolveRobotArmExtrinsicsModel2(model);

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
