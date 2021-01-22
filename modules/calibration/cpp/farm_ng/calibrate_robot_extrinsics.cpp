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
#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

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
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(raw_residuals);
    Eigen::Map<Sophus::SE3<T> const> b_pose_c(raw_b_pose_c);
    Eigen::Map<Sophus::SE3<T> const> a_pose_d(raw_a_pose_d);
    Sophus::SE3<T> c_i_pose_c_j = c_i_pose_c_j_.cast<T>();
    Sophus::SE3<T> b_i_pose_b_j = b_i_pose_b_j_.cast<T>();
    Sophus::SE3<T> a_pose_a = a_pose_d * c_pose_d_.inverse().cast<T>() *
                              b_pose_c.inverse() *
                              a_pose_b_.inverse().cast<T>();
    residuals = a_pose_a.log();
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

  for (size_t i = 0; i < a_poses_b.size() - 1; ++i) {
    Sophus::SE3d a_i_pose_b_i = a_poses_b[i];
    Sophus::SE3d c_i_pose_d_i = c_poses_d[i];

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PoseCostFunction, 6,
                                        Sophus::SE3d::num_parameters>(
            new PoseCostFunction(a_poses_b[i], c_poses_d[i]));

    problem.AddResidualBlock(cost_function, nullptr, b_pose_c->data(),
                             a_pose_d->data());
  }
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
}

RobotExtrinsicsModel SolveRobotExtrinsicsModel(RobotExtrinsicsModel model) {
  CalibrateMultiViewApriltagRigResult result;

  result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

  MultiViewApriltagRigModel camera_tag_model;
  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);
    camera_tag_model.add_multi_view_detections()->CopyFrom(
        measurement.multi_view_detections());
  }
  auto tag_rig = TagRigFromMultiViewDetections(
      model.link_tag_rig().name(), model.link_tag_rig().root_tag_id(),
      &camera_tag_model);

  CameraRigFromMultiViewDetections(model.base_camera_rig().name(),
                                   model.base_camera_rig().root_camera_name(),
                                   tag_rig, &camera_tag_model);
  camera_tag_model.set_solver_status(SolverStatus::SOLVER_STATUS_INITIAL);
  ModelError(&camera_tag_model);
  camera_tag_model = SolveMultiViewApriltagModel(camera_tag_model);
  result.mutable_multi_view_apriltag_rig_solved()->CopyFrom(
      core::ArchiveProtobufAsBinaryResource("solved", camera_tag_model));
  result.set_rmse(camera_tag_model.rmse());
  result.set_solver_status(camera_tag_model.solver_status());
  result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
  core::ArchiveProtobufAsJsonResource(model.base_camera_rig().name(), result);
  core::WriteProtobufAsJsonResource(core::BUCKET_APRILTAG_RIG_MODELS,
                                    model.base_camera_rig().name(), result);
  model.mutable_base_camera_rig_model()->CopyFrom(camera_tag_model);

  std::vector<Sophus::SE3d> camera_rig_poses_tag_rig, link_poses_base;
  perception::PoseGraph pose_graph;
  std::string camera_rig_name = model.base_camera_rig().name();
  for (const NamedSE3Pose& camera_rig_pose_apriltag_rig :
       camera_tag_model.camera_rig_poses_apriltag_rig()) {
    pose_graph.AddPose(camera_rig_pose_apriltag_rig);
  }
  for (int frame_num = 0; frame_num < model.measurements_size(); ++frame_num) {
    const RobotExtrinsicsModel::Measurement& measurement =
        model.measurements(frame_num);

    std::string rig_frame_name = camera_tag_model.apriltag_rig().name() +
                                 "/view/" + std::to_string(frame_num);
    if (!pose_graph.HasEdge(camera_rig_name, rig_frame_name)) {
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

    LOG(INFO) <<

        "camera_pose_tag: t: " << camera_pose_tag->translation().transpose()
              << " r: " << camera_pose_tag->unit_quaternion().vec().transpose();
    LOG(INFO) << "base_pose_tag: t: "
              << base_pose_link->translation().transpose()
              << " r: " << base_pose_link->unit_quaternion().vec().transpose();

    camera_rig_poses_tag_rig.push_back(*camera_pose_tag);
    link_poses_base.push_back(base_pose_link->inverse());
  }
  Sophus::SE3 tag_rig_pose_link = Sophus::SE3d::rotX(0);
  Sophus::SE3 base_pose_camera_rig = Sophus::SE3d::rotX(0);

  DualPoseEstimate(camera_rig_poses_tag_rig, link_poses_base,
                   &tag_rig_pose_link);

  DualPoseEstimate(link_poses_base, camera_rig_poses_tag_rig,
                   &base_pose_camera_rig);

  LOG(INFO) << "link_pose_tag_rig: "
            << tag_rig_pose_link.inverse().translation().transpose();
  LOG(INFO) << "base_pose_camera_rig: "
            << base_pose_camera_rig.translation().transpose();

  return model;
}

RobotExtrinsicsModel SolveRobotExtrinsicsModel2(RobotExtrinsicsModel model) {
  perception::PoseGraph pose_graph;
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

      for (const auto& detection : detections_per_view.detections()) {
        std::string tag_frame =
            perception::FrameRigTag(tag_rig_frame, detection.id());
        perception::PoseEdge* tag_to_tag_rig =
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
            cost_function1, new ceres::HuberLoss(2.0),
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

  // for (auto pose : pose_graph.ToNamedSE3Poses()) {
  //   LOG(INFO) << pose.ShortDebugString();
  // }
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
