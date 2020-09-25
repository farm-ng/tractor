#include "farm_ng/ipc.h"

#include <future>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>

#include "farm_ng/event_log_reader.h"
#include "farm_ng/sophus_protobuf.h"
#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/geometry.pb.h"

#include <sophus/average.hpp>
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>

DEFINE_string(log, "", "Path to log file, recorded with farm-ng-ipc-logger");
DEFINE_string(out_archive, "default",
              "When running from a log, what archive name should we write to?");

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetection;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::ApriltagRig;
using farm_ng_proto::tractor::v1::ApriltagRigTagStats;
using farm_ng_proto::tractor::v1::CalibratorCommand;
using farm_ng_proto::tractor::v1::CalibratorStatus;
using farm_ng_proto::tractor::v1::CameraModel;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::LoggingCommand;
using farm_ng_proto::tractor::v1::LoggingStatus;
using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::SolverStatus;

using Sophus::SE3d;
using Sophus::Vector6d;

namespace farm_ng {

// Constructs a frame name, using the convention <name>/<number %05d>
std::string FrameNameNumber(const std::string& name, int number) {
  char buffer[1024];
  CHECK_LT(
      std::snprintf(buffer, sizeof(buffer), "%s/%05d", name.c_str(), number),
      int(sizeof(buffer)));
  return std::string(buffer);
}

// Constructs a frame name of a tag associated with a rig in the form
// <rig_name>/tag/<tag_id %05d>
std::string FrameRigTag(const std::string& rig_name, int tag_id) {
  return FrameNameNumber(rig_name + "/tag", tag_id);
}

// Given a point in 3D space, compute the corresponding pixel coordinates in an
//  image with no distortion or forward distortion coefficients produced by the
//  same camera
//
//  This is compatable with autodiff using ceres jet types, except that it will
//  not support solving for the camera model itself.
template <class T>
Eigen::Matrix<T, 2, 1> ProjectPointToPixel(
    const CameraModel& camera, const Eigen::Matrix<T, 3, 1>& point) {
  using std::atan;
  using std::sqrt;
  const T eps(std::numeric_limits<float>::epsilon());
  T x = point.x() / point.z();

  T y = point.y() / point.z();

  CHECK_EQ(camera.distortion_model(),
           CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4);
  if (camera.distortion_model() ==
      CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4) {
    // Model copied from librealsense:
    // https://github.com/IntelRealSense/librealsense/blob/0adceb9dc6fce63c348346e1aef1b63c052a1db9/include/librealsense2/rsutil.h#L63
    T r = sqrt(x * x + y * y);
    if (r < eps) {
      r = eps;
    }
    T theta = atan(r);
    T theta2 = theta * theta;
    T series =
        T(1) +
        theta2 *
            (T(camera.distortion_coefficients(0)) +
             theta2 *
                 (T(camera.distortion_coefficients(1)) +
                  theta2 * (T(camera.distortion_coefficients(2)) +
                            theta2 * T(camera.distortion_coefficients(3)))));
    T rd = theta * series;
    x *= rd / r;
    y *= rd / r;
  }

  return Eigen::Matrix<T, 2, 1>(x * T(camera.fx()) + T(camera.cx()),
                                y * T(camera.fy()) + T(camera.cy()));
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

// https://github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
class LocalParameterizationSE3 : public ceres::LocalParameterization {
 public:
  virtual ~LocalParameterizationSE3() {}

  // SE3 plus operation for Ceres
  //
  //  T * exp(x)
  //
  virtual bool Plus(double const* T_raw, double const* delta_raw,
                    double* T_plus_delta_raw) const {
    Eigen::Map<SE3d const> const T(T_raw);
    Eigen::Map<Vector6d const> const delta(delta_raw);
    Eigen::Map<SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta = T * SE3d::exp(delta);
    return true;
  }

  // Jacobian of SE3 plus operation for Ceres
  //
  // Dx T * exp(x)  with  x=0
  //
  virtual bool ComputeJacobian(double const* T_raw,
                               double* jacobian_raw) const {
    Eigen::Map<SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
        jacobian_raw);
    jacobian = T.Dx_this_mul_exp_x_at_0();
    return true;
  }

  virtual int GlobalSize() const { return SE3d::num_parameters; }

  virtual int LocalSize() const { return SE3d::DoF; }
};

bool StartsWith(const std::string& x, const std::string& prefix) {
  return x.rfind(prefix, 0) == 0;
}

std::array<Eigen::Vector3d, 4> PointsTag(const ApriltagDetection& detection) {
  CHECK(detection.tag_size() > 0.0);
  double half_size = detection.tag_size() / 2.0;
  return std::array<Eigen::Vector3d, 4>(
      {Eigen::Vector3d(-half_size, -half_size, 0),
       Eigen::Vector3d(half_size, -half_size, 0.0),
       Eigen::Vector3d(half_size, half_size, 0.0),
       Eigen::Vector3d(-half_size, half_size, 0.0)});
}

std::array<Eigen::Vector2d, 4> PointsImage(const ApriltagDetection& detection) {
  return std::array<Eigen::Vector2d, 4>(
      {Eigen::Vector2d(detection.p(0).x(), detection.p(0).y()),
       Eigen::Vector2d(detection.p(1).x(), detection.p(1).y()),
       Eigen::Vector2d(detection.p(2).x(), detection.p(2).y()),
       Eigen::Vector2d(detection.p(3).x(), detection.p(3).y())});
}

struct ApriltagRigModel {
  int root_id;
  std::string rig_name;
  std::string camera_frame_name;
  std::vector<ApriltagDetections> all_detections;
  std::unordered_map<int, SE3d> tag_pose_root;
  std::unordered_map<int, double> tag_size;
  std::unordered_map<int, std::array<Eigen::Vector3d, 4>> points_tag;
  std::vector<Sophus::optional<SE3d>> camera_poses_root;

  std::vector<Image> reprojection_images;

  SolverStatus status;
  std::unordered_map<int, ApriltagRigTagStats> per_tag_stats;
  double rmse;

  void ToMonocularApriltagRigModel(MonocularApriltagRigModel* rig) const {
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
};

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
      per_tag_stats.mutable_per_image_rmse()->insert(
          {int(frame_n), std::sqrt(tag_error)});
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

class ApriltagRigCalibrator {
 public:
  ApriltagRigCalibrator(EventBus* bus,
                        const CalibratorCommand::ApriltagRigStart& rig_start)
      : bus_(bus), root_id_(0) {
    if (rig_start.tag_ids_size() > 0) {
      root_id_ = rig_start.tag_ids().Get(0);
    }
    for (auto id : rig_start.tag_ids()) {
      ids_.insert(id);
    }
  }

  void PoseInit(const std::vector<std::unordered_map<int, SE3d>>& frames,
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
          LOG(WARNING) << "failed to average camera pose for frame: "
                       << frame_n;
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
                << " q:"
                << o_tag_pose_root->unit_quaternion().vec().transpose();
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

  ApriltagRigModel PoseInitialization() {
    std::vector<std::unordered_map<int, SE3d>> frames;

    std::string camera_frame_name;

    std::unordered_map<int, double> tag_size;
    std::unordered_map<int, std::array<Eigen::Vector3d, 4>> points_tag;

    for (const auto& detections : all_detections_) {
      std::unordered_map<int, SE3d> camera_pose_tags;
      if (camera_frame_name.empty()) {
        camera_frame_name = detections.image().camera_model().frame_name();
      }
      CHECK_EQ(camera_frame_name,
               detections.image().camera_model().frame_name());
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
          CHECK(false) << "Malformed apriltag : "
                       << detection.ShortDebugString();
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

  void AddFrame(ApriltagDetections detections) {
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

  int NumFrames() const { return static_cast<int>(all_detections_.size()); }

  EventBus* bus_;
  std::string rig_name_ = "rig";  // TODO(ethanrublee) set from command/config.
  int root_id_;
  std::unordered_set<int> ids_;
  std::vector<ApriltagDetections> all_detections_;
};  // namespace farm_ng

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

class Calibrator {
 public:
  Calibrator(EventBus& bus)
      : bus_(bus),
        timer_(bus_.get_io_service()),
        apriltag_rig_calibrator_(&bus_, CalibratorCommand::ApriltagRigStart()),
        bus_connection_(bus_.GetEventSignal()->connect(
            std::bind(&Calibrator::on_event, this, std::placeholders::_1))) {
    on_timer(boost::system::error_code());
  }

  boost::signals2::connection& bus_connection() { return bus_connection_; }

  void send_status() { bus_.Send(MakeEvent("calibrator/status", status_)); }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(
        std::bind(&Calibrator::on_timer, this, std::placeholders::_1));

    send_status();
  }

  bool is_calibration_event(const std::string& s) {
    return s.rfind("calibrator/", 0) == 0;
  }

  void Solve() {
    status_.mutable_apriltag_rig()->set_finished(false);
    LOG(INFO) << "Initial reprojection error vvvvvvvv";
    ApriltagRigModel model = apriltag_rig_calibrator_.PoseInitialization();
    MonocularApriltagRigModel model_pb;
    model.ToMonocularApriltagRigModel(&model_pb);

    status_.mutable_apriltag_rig()->mutable_rig_model_resource()->CopyFrom(
        WriteProtobufToJsonResource("apriltag_rig_model/initial", model_pb));

    send_status();
    LOG(INFO) << "Initial reprojection error ^^^^^^^";

    farm_ng::Solve(model);
    model.ToMonocularApriltagRigModel(&model_pb);
    status_.mutable_apriltag_rig()->mutable_rig_model_resource()->CopyFrom(
        WriteProtobufToJsonResource("apriltag_rig_model/solved", model_pb));

    status_.mutable_apriltag_rig()->set_finished(true);
    send_status();
  }

  bool on_calibrator_command(const EventPb& event) {
    if (!event.data().UnpackTo(&command_)) {
      return false;
    }
    if (command_.has_apriltag_rig_start()) {
      apriltag_rig_calibrator_ =
          ApriltagRigCalibrator(&bus_, command_.apriltag_rig_start());
      status_.mutable_apriltag_rig()->Clear();
      send_status();
    }
    if (command_.has_solve()) {
      Solve();
    }
    return true;
  }

  bool on_apriltag_detections(const EventPb& event) {
    ApriltagDetections detections;
    if (!event.data().UnpackTo(&detections)) {
      return false;
    }
    VLOG(2) << detections.ShortDebugString();
    // check if we're calibrating a rig?
    if (status_.has_apriltag_rig()) {
      apriltag_rig_calibrator_.AddFrame(detections);
      status_.mutable_apriltag_rig()->set_num_frames(
          apriltag_rig_calibrator_.NumFrames());
      send_status();
    }
    return true;
  }

  void on_event(const EventPb& event) {
    if (!is_calibration_event(event.name())) {
      return;
    }
    if (on_calibrator_command(event)) {
    } else if (on_apriltag_detections(event)) {
    }
  }

  EventBus& bus() { return bus_; }

 private:
  EventBus& bus_;

  boost::asio::deadline_timer timer_;

  CalibratorCommand command_;
  CalibratorStatus status_;

  ApriltagRigCalibrator apriltag_rig_calibrator_;
  boost::signals2::connection bus_connection_;
};

void WaitForServices(EventBus& bus,
                     const std::vector<std::string>& service_names) {
  LOG(INFO) << "Waiting for services: ";
  for (const auto& name : service_names) {
    LOG(INFO) << "   " << name;
  }
  bool has_all = false;
  while (!has_all) {
    std::vector<bool> has_service(service_names.size(), false);
    for (const auto& announce : bus.GetAnnouncements()) {
      for (size_t i = 0; i < service_names.size(); ++i) {
        if (announce.second.service() == service_names[i]) {
          has_service[i] = true;
        }
      }
    }
    has_all = true;
    for (auto x : has_service) {
      has_all &= x;
    }
    bus.get_io_service().poll();
  }
}

LoggingStatus WaitForLoggerStatus(
    EventBus& bus, std::function<bool(const LoggingStatus&)> predicate) {
  LoggingStatus status;
  while (true) {
    bus.get_io_service().run_one();
    if (bus.GetState().count("logger/status") &&
        bus.GetState().at("logger/status").data().UnpackTo(&status) &&
        predicate(status)) {
      LOG(INFO) << "Logger status: " << status.ShortDebugString();
      return status;
    }
  }
}

LoggingStatus WaitForLoggerStart(EventBus& bus,
                                 const std::string& archive_path) {
  return WaitForLoggerStatus(bus, [archive_path](const LoggingStatus& status) {
    return (status.has_recording() &&
            status.recording().archive_path() == archive_path);
  });
}

LoggingStatus WaitForLoggerStop(EventBus& bus) {
  return WaitForLoggerStatus(bus, [](const LoggingStatus& status) {
    return (status.state_case() == LoggingStatus::kStopped);
  });
}

LoggingStatus StartLogging(EventBus& bus, const std::string& archive_path) {
  WaitForLoggerStop(bus);
  LoggingCommand command;
  command.mutable_record_start()->set_archive_path(archive_path);
  bus.Send(farm_ng::MakeEvent("logger/command", command));
  return WaitForLoggerStart(bus, archive_path);
}

LoggingStatus StopLogging(EventBus& bus) {
  LoggingCommand command;
  command.mutable_record_stop();
  bus.Send(farm_ng::MakeEvent("logger/command", command));
  return WaitForLoggerStop(bus);
}

}  // namespace farm_ng

int main(int argc, char* argv[]) {
  // Initialize Google's logging library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  boost::asio::io_service io_service;
  farm_ng::EventBus& bus = farm_ng::GetEventBus(io_service, "calibrator");
  farm_ng::Calibrator calibrator(bus);

  // we're reading from a log, so block event_bus events from reaching the
  // calibator.
  bool should_block = !FLAGS_log.empty();
  boost::signals2::shared_connection_block block_external_events(
      calibrator.bus_connection(), should_block);

  farm_ng::WaitForServices(bus, {"ipc-logger", "calibrator"});

  if (!FLAGS_log.empty()) {
    farm_ng::StartLogging(bus, FLAGS_out_archive);
    LOG(INFO) << "Using log : " << FLAGS_log;
    farm_ng::EventLogReader log_reader(FLAGS_log);
    while (true) {
      EventPb event;
      try {
        io_service.poll();
        event = log_reader.ReadNext();
        calibrator.on_event(event);
      } catch (std::runtime_error& e) {
        break;
      }
    }
    farm_ng::StopLogging(bus);
  } else {
    io_service.run();
  }
  return 0;
}
