#ifndef FARM_NG_PERCEPTION_POSE_GRAPH_H_
#define FARM_NG_PERCEPTION_POSE_GRAPH_H_
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <sophus/average.hpp>
#include <sophus/se3.hpp>

#include "farm_ng/perception/geometry.pb.h"
#include "farm_ng/perception/sophus_protobuf.h"

namespace farm_ng {
namespace perception {

using Sophus::SE3d;

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

  bool HasEdge(const std::string& frame_a, const std::string& frame_b) const {
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
    if (frame_a == frame_b) {
      return SE3d::rotX(0.0);
    }
    if (!HasEdge(frame_a, frame_b)) {
      LOG(WARNING) << "No edge between: " << frame_a << " <-> " << frame_b;
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
}  // namespace perception
}  // namespace farm_ng
#endif
