#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

#include <ceres/ceres.h>
#include <opencv2/highgui.hpp>  // TODO remove.
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <google/protobuf/util/time_util.h>
#include <sophus/average.hpp>

#include "farm_ng/blobstore.h"

#include "farm_ng/event_log_reader.h"
#include "farm_ng/image_utils.h"
#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

#include "farm_ng/calibration/apriltag.h"
#include "farm_ng/calibration/camera_model.h"

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/pose_utils.h"
#include "farm_ng/calibration/time_series.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"

#include "farm_ng/image_loader.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

namespace farm_ng {

struct pose_edge_t {
  typedef boost::edge_property_tag kind;
};

struct SE3Map {
  bool invert;

  template <typename Scalar>
  Sophus::SE3<Scalar> Map(const Sophus::SE3<Scalar>& pose) const {
    if (invert) {
      return pose.inverse();
    }
    return pose;
  }

  template <typename Scalar>
  Sophus::SE3<Scalar> Map(const Scalar* raw_pose) const {
    Eigen::Map<Sophus::SE3<Scalar> const> map_pose(raw_pose);
    if (invert) {
      return map_pose.inverse();
    } else {
      return map_pose;
    }
  }
};

struct PoseEdge {
  std::vector<Sophus::SE3d> a_poses_b;
  std::optional<Sophus::SE3d> a_pose_b;
  std::string frame_a;
  std::string frame_b;

  Sophus::SE3d& GetAPoseB() {
    if (!a_pose_b) {
      auto o_pose = Sophus::average(a_poses_b);
      CHECK(o_pose);
      a_pose_b = *o_pose;
    }
    return *a_pose_b;
  }
  const Sophus::SE3d& GetAPoseB() const {
    return const_cast<PoseEdge*>(this)->GetAPoseB();
  }

  SE3Map GetAPoseBMap(const std::string& frame_a,
                      const std::string& frame_b) const {
    SE3Map map;
    map.invert = frame_a != this->frame_a;
    if (map.invert) {
      CHECK_EQ(frame_b, this->frame_a);
      CHECK_EQ(frame_a, this->frame_b);
    } else {
      CHECK_EQ(frame_a, this->frame_a);
      CHECK_EQ(frame_b, this->frame_b);
    }
    return map;
  }

  SE3d GetAPoseBMapped(const std::string& frame_a,
                       const std::string& frame_b) const {
    return GetAPoseBMap(frame_a, frame_b).Map(GetAPoseB());
  }

  friend std::ostream& operator<<(std::ostream& out, const PoseEdge& self) {
    out << self.frame_a << " <- " << self.frame_b;
    if (self.a_pose_b) {
      out << " t: " << self.a_pose_b->translation().transpose()
          << " r:" << self.a_pose_b->unit_quaternion().vec().transpose();
    }
    return out;
  }
};

class PoseGraph {
 public:
  typedef boost::property<boost::vertex_name_t, std::string> VertexProperty;

  typedef boost::property<pose_edge_t, PoseEdge,
                          boost::property<boost::edge_weight_t, float>>
      EdgeProperty;
  typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                                VertexProperty, EdgeProperty>
      GraphT;

  bool HasName(const std::string& frame_name) const {
    return name_id_.count(frame_name) != 0;
  }
  size_t MakeId(const std::string& frame_name) {
    auto it = name_id_.find(frame_name);
    if (it != name_id_.end()) {
      return it->second;
    }

    size_t id = boost::add_vertex(graph_);

    CHECK(name_id_.insert(std::make_pair(frame_name, id)).second);
    CHECK(id_name_.insert(std::make_pair(id, frame_name)).second);
    boost::put(VertexNameMap(), id, frame_name);
    return id;
  }
  boost::property_map<GraphT, boost::vertex_name_t>::type VertexNameMap() {
    return boost::get(boost::vertex_name_t(), graph_);
  }
  boost::property_map<GraphT, boost::vertex_name_t>::const_type VertexNameMap()
      const {
    return boost::get(boost::vertex_name_t(), graph_);
  }

  boost::property_map<GraphT, pose_edge_t>::type PoseEdgeMap() {
    return boost::get(pose_edge_t(), graph_);
  }

  boost::property_map<GraphT, pose_edge_t>::const_type PoseEdgeMap() const {
    return boost::get(pose_edge_t(), graph_);
  }

  boost::property_map<GraphT, boost::edge_weight_t>::type EdgeWeightMap() {
    return boost::get(boost::edge_weight_t(), graph_);
  }

  boost::property_map<GraphT, boost::edge_weight_t>::const_type EdgeWeightMap()
      const {
    return boost::get(boost::edge_weight_t(), graph_);
  }

  size_t GetId(const std::string& frame_name) const {
    auto it = name_id_.find(frame_name);
    CHECK(it != name_id_.end()) << "No frame named: " << frame_name;
    return it->second;
  }

  std::string GetName(size_t frame_id) const {
    auto it = id_name_.find(frame_id);
    CHECK(it != id_name_.end()) << "No frame id: " << frame_id;
    return it->second;
  }
  std::pair<GraphT::edge_iterator, GraphT::edge_iterator> Edges() const {
    return boost::edges(graph_);
  }

  std::vector<PoseEdge*> MutablePoseEdges() {
    std::vector<PoseEdge*> pose_edges;
    auto edge_map = PoseEdgeMap();
    for (auto es = Edges(); es.first != es.second; ++es.first) {
      pose_edges.push_back(&edge_map[*es.first]);
    }
    return pose_edges;
  }

  bool HasEdge(const std::string& frame_a, const std::string& frame_b) {
    if (!HasName(frame_a) || !HasName(frame_b)) {
      return false;
    }
    size_t id_a = GetId(frame_a);
    size_t id_b = GetId(frame_b);
    if (id_a >= id_b) {
      std::swap(id_a, id_b);
    }
    auto edge = boost::edge(id_a, id_b, graph_);
    return edge.second;
  }

  PoseEdge* MutablePoseEdge(const std::string& frame_a,
                            const std::string& frame_b) {
    size_t id_a = GetId(frame_a);
    size_t id_b = GetId(frame_b);
    if (id_a >= id_b) {
      std::swap(id_a, id_b);
    }
    auto edge = boost::edge(id_a, id_b, graph_);
    CHECK(edge.second);
    return &PoseEdgeMap()[edge.first];
  }

  void AddPose(std::string frame_a, std::string frame_b, SE3d a_pose_b) {
    CHECK_NE(frame_a, frame_b);
    size_t id_a = MakeId(frame_a);
    size_t id_b = MakeId(frame_b);

    if (id_a >= id_b) {
      std::swap(id_a, id_b);
      std::swap(frame_a, frame_b);
      a_pose_b = a_pose_b.inverse();
    }
    auto edge_exists = boost::add_edge(id_a, id_b, graph_);
    PoseEdge& pose_edge = PoseEdgeMap()[edge_exists.first];
    if (pose_edge.a_poses_b.empty()) {
      pose_edge.a_pose_b = a_pose_b;
      pose_edge.frame_a = frame_a;
      pose_edge.frame_b = frame_b;
    } else {
      pose_edge.a_pose_b.reset();
      CHECK_EQ(pose_edge.frame_a, frame_a);
      CHECK_EQ(pose_edge.frame_b, frame_b);
    }
    pose_edge.a_poses_b.push_back(a_pose_b);

    EdgeWeightMap()[edge_exists.first] =
        1.0 / PoseEdgeMap()[edge_exists.first].a_poses_b.size();
    VLOG(3) << frame_a << " <-> " << frame_b
            << " weight: " << EdgeWeightMap()[edge_exists.first];
  }
  void AddPose(const NamedSE3Pose& pose) {
    SE3d a_pose_b;
    ProtoToSophus(pose.a_pose_b(), &a_pose_b);
    AddPose(pose.frame_a(), pose.frame_b(), a_pose_b);
  }

  std::vector<size_t> ComputeShortestPaths(std::string frame_a) const {
    size_t id_a = GetId(frame_a);
    auto weights = EdgeWeightMap();
    std::vector<size_t> p(boost::num_vertices(graph_));
    std::vector<float> d(boost::num_vertices(graph_));

    boost::dijkstra_shortest_paths(
        graph_, id_a,
        boost::predecessor_map(
            boost::make_iterator_property_map(
                p.begin(), boost::get(boost::vertex_index, graph_)))
            .distance_map(boost::make_iterator_property_map(
                d.begin(), boost::get(boost::vertex_index, graph_))));

    return p;
  }

  SE3d AverageAPoseB(size_t frame_a, size_t frame_b) const {
    if (frame_a == frame_b) {
      return SE3d::rotX(0);
    }
    bool invert = false;
    if (frame_a >= frame_b) {
      std::swap(frame_a, frame_b);
      invert = true;
    }
    auto edge = boost::edge(frame_a, frame_b, graph_);
    CHECK(edge.second);
    auto pose = PoseEdgeMap()[edge.first].GetAPoseB();
    if (invert) {
      return pose.inverse();
    }
    return pose;
  }

  std::optional<SE3d> AverageAPoseB(const std::string& frame_a,
                                    const std::string& frame_b) const {
    if (!HasName(frame_a)) {
      LOG(WARNING) << frame_a << " isn't in graph.";
      return std::optional<SE3d>();
    }
    if (!HasName(frame_b)) {
      LOG(WARNING) << frame_b << " isn't in graph.";
      return std::optional<SE3d>();
    }

    return AverageAPoseB(GetId(frame_a), GetId(frame_b));
  }

  std::optional<SE3d> AverageAPoseB(size_t id_a, size_t id_b,
                                    const std::vector<size_t>& p) const {
    if (id_a == id_b) {
      return SE3d::rotX(0);
    }
    size_t n = id_b;
    SE3d b_pose_a = SE3d::rotX(0);
    while (n != id_a) {
      size_t child = n;
      size_t parent = p[n];
      if (parent == child) {
        LOG(INFO) << "no parent: " << GetName(child);
        return std::optional<SE3d>();
      }

      n = parent;
      auto child_pose_parent = AverageAPoseB(child, parent);
      b_pose_a = b_pose_a * child_pose_parent;
    }
    return b_pose_a.inverse();
  }

  std::optional<SE3d> AverageAPoseB(std::string frame_a, std::string frame_b,
                                    const std::vector<size_t>& p) const {
    if (frame_a == frame_b) {
      return SE3d::rotX(0);
    }
    CHECK(name_id_.count(frame_a) != 0) << frame_a;
    CHECK(name_id_.count(frame_b) != 0) << frame_a;
    size_t id_b = name_id_.at(frame_b);
    size_t id_a = name_id_.at(frame_b);
    return AverageAPoseB(id_a, id_b, p);
  }

  // This function computes the shortest path (weighted inversely by number of
  // poses between frames) of very frame to the given frame_a, and then
  // collapses each path in to a single SE3 transform, such that the returned
  // posegraph contains only edges which are between frame_a and frame_X, and
  // each edge contains only a single pose.
  PoseGraph AveragePoseGraph(std::string frame_a) const {
    auto p = ComputeShortestPaths(frame_a);
    size_t id_a = GetId(frame_a);

    PoseGraph pose_graph;
    for (size_t id_x = 0; id_x < p.size(); ++id_x) {
      if (id_x == id_a) {
        continue;
      }
      auto o_a_pose_x = AverageAPoseB(id_a, id_x, p);

      if (!o_a_pose_x) {
        continue;
      }
      pose_graph.AddPose(frame_a, id_name_.at(id_x), *o_a_pose_x);
    }
    return pose_graph;
  }

  google::protobuf::RepeatedPtrField<NamedSE3Pose> ToNamedSE3Poses() const {
    google::protobuf::RepeatedPtrField<NamedSE3Pose> poses;
    for (auto es = Edges(); es.first != es.second; es.first++) {
      auto edge = PoseEdgeMap()[*es.first];
      SophusToProto(edge.GetAPoseB(), edge.frame_a, edge.frame_b, poses.Add());
    }
    return poses;
  }
  void UpdateNamedSE3Pose(NamedSE3Pose* pose) const {
    auto a_pose_b = AverageAPoseB(pose->frame_a(), pose->frame_b());

    CHECK(a_pose_b);
    SophusToProto(*a_pose_b, pose->mutable_a_pose_b());
  }
  void UpdateNamedSE3Poses(
      google::protobuf::RepeatedPtrField<NamedSE3Pose>* poses) const {
    for (NamedSE3Pose& pose : *poses) {
      UpdateNamedSE3Pose(&pose);
    }
  }

 private:
  std::unordered_map<std::string, size_t> name_id_;
  std::unordered_map<size_t, std::string> id_name_;

  GraphT graph_;
};

using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::Event;
using farm_ng_proto::tractor::v1::MultiViewApriltagDetections;
using farm_ng_proto::tractor::v1::PerImageRmse;
using Sophus::SE3d;

struct CameraRigApriltagRigCostFunctor {
  CameraRigApriltagRigCostFunctor(const CameraModel& camera,
                                  std::array<Eigen::Vector3d, 4> points_tag,
                                  std::array<Eigen::Vector2d, 4> points_image,
                                  SE3Map camera_pose_camera_rig,
                                  SE3Map tag_rig_pose_tag,
                                  SE3Map camera_rig_pose_tag_rig)
      : camera_(camera),
        points_tag_(points_tag),
        points_image_(points_image),
        camera_pose_camera_rig_(camera_pose_camera_rig),
        tag_rig_pose_tag_(tag_rig_pose_tag),
        camera_rig_pose_tag_rig_(camera_rig_pose_tag_rig) {}

  template <class T>
  Eigen::Matrix<T, 4, 2> Project(
      T const* const raw_camera_pose_camera_rig,
      T const* const raw_tag_rig_pose_tag,
      T const* const raw_camera_rig_pose_tag_rig) const {
    auto camera_pose_camera_rig =
        camera_pose_camera_rig_.Map(raw_camera_pose_camera_rig);
    auto tag_rig_pose_tag = tag_rig_pose_tag_.Map(raw_tag_rig_pose_tag);
    auto camera_rig_pose_tag_rig =
        tag_rig_pose_tag_.Map(raw_camera_rig_pose_tag_rig);
    Sophus::SE3<T> camera_pose_tag =
        camera_pose_camera_rig * camera_rig_pose_tag_rig * tag_rig_pose_tag;

    Eigen::Matrix<T, 4, 2> points_image;
    for (int i = 0; i < 4; ++i) {
      points_image.row(i) = ProjectPointToPixel(
          camera_, camera_pose_tag * points_tag_[i].cast<T>());
    }
    return points_image;
  }

  template <class T>
  bool operator()(T const* const raw_camera_pose_camera_rig,
                  T const* const raw_tag_rig_pose_tag,
                  T const* const raw_camera_rig_pose_tag_rig,
                  T* raw_residuals) const {
    Eigen::Map<Eigen::Matrix<T, 4, 2>> residuals(raw_residuals);

    Eigen::Matrix<T, 4, 2> points_image =
        Project(raw_camera_pose_camera_rig, raw_tag_rig_pose_tag,
                raw_camera_rig_pose_tag_rig);

    for (int i = 0; i < 4; ++i) {
      residuals.row(i) =
          points_image_[i].cast<T>() - points_image.row(i).transpose();
    }
    return true;
  }
  CameraModel camera_;
  std::array<Eigen::Vector3d, 4> points_tag_;
  std::array<Eigen::Vector2d, 4> points_image_;
  SE3Map camera_pose_camera_rig_;
  SE3Map tag_rig_pose_tag_;
  SE3Map camera_rig_pose_tag_rig_;
};
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
        tag_stats[detection.id()].set_n_frames(
            tag_stats[detection.id()].n_frames() + 1);
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
    reprojection_image.mutable_camera_model()->set_image_width(1280 * 3);
    reprojection_image.mutable_camera_model()->set_image_height(720 * 2);
    auto resource_path = GetUniqueArchiveResource(
        FrameNameNumber(
            "reprojection-" + farm_ng_proto::tractor::v1::SolverStatus_Name(
                                  model->solver_status()),
            frame_num),
        "png", "image/png");
    reprojection_image.mutable_resource()->CopyFrom(resource_path.first);
    LOG(INFO) << resource_path.second.string();
    CHECK(
        cv::imwrite(resource_path.second.string(),
                    ConstructGridImage(images, cv::Size(1280 * 3, 720 * 2), 3)))
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
      google::protobuf::util::TimeUtil::MillisecondsToDuration(1000.0 / 7);

  std::map<std::string, int> detection_counts;

  for (const Event& event : apriltag_series[root_camera_name + "/apriltags"]) {
    ApriltagDetections detections;
    CHECK(event.data().UnpackTo(&detections));
    if (tag_filter.AddApriltags(detections)) {
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
PoseGraph TagRigFromMultiViewDetections(
    const CalibrateMultiViewApriltagRigConfiguration& config,
    MultiViewApriltagRigModel* model) {
  model->mutable_apriltag_rig()->set_name(config.tag_rig_name());
  model->mutable_apriltag_rig()->set_root_tag_id(config.root_tag_id());

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
        node.set_frame_name(FrameRigTag(config.tag_rig_name(), detection.id()));

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
  std::string root_tag = "tag/" + std::to_string(config.root_tag_id());

  auto tag_rig_small = tag_rig.AveragePoseGraph(root_tag);

  for (auto& node : tag_rig_nodes) {
    std::string tag = "tag/" + std::to_string(node.first);
    auto root_pose_tag = tag_rig_small.AverageAPoseB(root_tag, tag);
    if (!root_pose_tag) {
      continue;
    }
    SophusToProto(*root_pose_tag, config.tag_rig_name(),
                  node.second.frame_name(), node.second.mutable_pose());
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
  auto tag_rig = TagRigFromMultiViewDetections(config, &model);
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
}  // namespace farm_ng
