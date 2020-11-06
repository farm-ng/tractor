#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

#include <ceres/ceres.h>

#include <google/protobuf/util/time_util.h>
#include <sophus/average.hpp>

#include "farm_ng/blobstore.h"

#include "farm_ng/event_log_reader.h"
#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

#include "farm_ng/calibration/apriltag.h"

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/pose_utils.h"
#include "farm_ng/calibration/time_series.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

namespace farm_ng {

struct pose_edge_t {
  typedef boost::edge_property_tag kind;
};

struct PoseEdge {
  std::vector<Sophus::SE3d> poses;
  std::optional<Sophus::SE3d> average_pose;
  std::string frame_a;
  std::string frame_b;

  std::optional<Sophus::SE3d> GetAveragePose() const {
    if (!average_pose) {
      auto o_pose = Sophus::average(poses);
      if (o_pose) {
        const_cast<PoseEdge*>(this)->average_pose = *o_pose;
      }
    }
    return average_pose;
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
    if (pose_edge.poses.empty()) {
      pose_edge.average_pose = a_pose_b;
      pose_edge.frame_a = frame_a;
      pose_edge.frame_b = frame_b;
    } else {
      pose_edge.average_pose.reset();
      CHECK_EQ(pose_edge.frame_a, frame_a);
      CHECK_EQ(pose_edge.frame_b, frame_b);
    }
    pose_edge.poses.push_back(a_pose_b);

    EdgeWeightMap()[edge_exists.first] =
        1.0 / PoseEdgeMap()[edge_exists.first].poses.size();
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

  std::optional<SE3d> AverageAPoseB(size_t frame_a, size_t frame_b) const {
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
    auto pose = PoseEdgeMap()[edge.first].GetAveragePose();
    if (invert && pose) {
      return pose->inverse();
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
      auto o_child_pose_parent = AverageAPoseB(child, parent);
      if (!o_child_pose_parent) {
        LOG(INFO) << "No pose between: " << GetName(child) << " "
                  << GetName(parent);
        return std::optional<SE3d>();
      }
      b_pose_a = b_pose_a * (*o_child_pose_parent);
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
      auto pose = edge.GetAveragePose();
      if (!pose) {
        continue;
      }
      SophusToProto(*pose, edge.frame_a, edge.frame_b, poses.Add());
    }
    return poses;
  }

 private:
  std::unordered_map<std::string, size_t> name_id_;
  std::unordered_map<size_t, std::string> id_name_;

  GraphT graph_;

};  // namespace farm_ng

using Sophus::SE3d;

using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::Event;
using farm_ng_proto::tractor::v1::MultiViewApriltagDetections;

MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    const MultiViewApriltagRigModel& model) {
  return model;
}

std::vector<MultiViewApriltagDetections> LoadMultiViewApriltagDetections(
    const std::string& root_camera_name, const Resource& event_log) {
  EventLogReader log_reader(event_log);

  std::map<std::string, TimeSeries<Event>> apriltag_series;

  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (std::runtime_error& e) {
      break;
    }
    if (event.data().Is<ApriltagDetections>()) {
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
  std::string root_tag_rig_name =
      FrameRigTag(config.tag_rig_name(), config.root_tag_id());

  auto tag_rig_small = tag_rig.AveragePoseGraph(root_tag);

  for (auto& node : tag_rig_nodes) {
    std::string tag = "tag/" + std::to_string(node.first);
    auto root_pose_tag = tag_rig_small.AverageAPoseB(root_tag, tag);
    if (!root_pose_tag) {
      continue;
    }
    SophusToProto(*root_pose_tag, root_tag_rig_name, node.second.frame_name(),
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
    auto camera_rig_i =
        camera_rig_step_i.AveragePoseGraph(config.root_camera_name());

    for (auto es = camera_rig_i.Edges(); es.first != es.second; es.first++) {
      auto edge = camera_rig_i.PoseEdgeMap()[*es.first];
      CHECK(edge.GetAveragePose());

      std::string frame_a = edge.frame_a;
      std::string frame_b = edge.frame_b;
      if (frame_a == root_tag_name || frame_b == root_tag_name) {
        if (frame_a == root_tag_name) {
          frame_a =
              "view/" + std::to_string(frame_num) + "/" + config.tag_rig_name();
        }
        if (frame_b == root_tag_name) {
          frame_b =
              "view/" + std::to_string(frame_num) + "/" + config.tag_rig_name();
        }
        camera_rig_tags.AddPose(frame_a, frame_b, *edge.GetAveragePose());
      } else {
        camera_rig_inter.AddPose(frame_a, frame_b, *edge.GetAveragePose());
      }
    }
  }
  for (auto camera : camera_models) {
    model->mutable_camera_rig()->add_cameras()->CopyFrom(camera.second);
  }

  model->mutable_camera_rig()->mutable_camera_pose_rig()->CopyFrom(
      camera_rig_inter.AveragePoseGraph(config.root_camera_name())
          .ToNamedSE3Poses());

  model->mutable_camera_rig_poses_apriltag_rig()->CopyFrom(
      camera_rig_tags.AveragePoseGraph(config.root_camera_name())
          .ToNamedSE3Poses());
}

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config) {
  auto dataset_result = ReadProtobufFromResource<CaptureVideoDatasetResult>(
      config.video_dataset());
  MultiViewApriltagRigModel model;

  for (const auto& mv_detections : LoadMultiViewApriltagDetections(
           config.root_camera_name(), dataset_result.dataset())) {
    model.add_multi_view_detections()->CopyFrom(mv_detections);
  }
  auto tag_rig = TagRigFromMultiViewDetections(config, &model);
  CameraRigFromMultiViewDetections(config, tag_rig, &model);
  auto debug_model = model;
  debug_model.clear_multi_view_detections();
  LOG(INFO) << debug_model.DebugString();
  return model;
}
}  // namespace farm_ng
