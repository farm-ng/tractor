#include "farm_ng/ipc.h"

#include <iostream>

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "farm_ng/sophus_protobuf.h"
#include "farm_ng_proto/tractor/v1/geometry.pb.h"

#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>

using farm_ng_proto::tractor::v1::Event;
using farm_ng_proto::tractor::v1::NamedSE3Pose;

namespace farm_ng {

class ExtrinsicCalibrator {
 public:
  ExtrinsicCalibrator(boost::asio::io_service& io_service)
      : bus_(GetEventBus(io_service)), timer_(io_service) {
    bus_.GetEventSignal()->connect(
        std::bind(&ExtrinsicCalibrator::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  Event GenerateExtrinsicPoseEvent() {
    NamedSE3Pose base_pose_t265;
    base_pose_t265.set_frame_a("tractor/base");
    base_pose_t265.set_frame_b("tracking_camera/front");
    Sophus::SE3d se3;
    se3.translation().x() = 0.20;
    se3.translation().z() = 0.20;
    SophusToProto(se3, base_pose_t265.mutable_a_pose_b());
    Event event =
        farm_ng::MakeEvent("pose/extrinsic_calibrator", base_pose_t265);
    return event;
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(200));
    timer_.async_wait(
        std::bind(&ExtrinsicCalibrator::on_timer, this, std::placeholders::_1));
    bus_.Send(GenerateExtrinsicPoseEvent());
  }

  void on_event(const farm_ng_proto::tractor::v1::Event& event) {
    if (event.data().type_url() !=
        "type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose") {
      return;
    }
    farm_ng_proto::tractor::v1::NamedSE3Pose pose;
    CHECK(event.data().UnpackTo(&pose));
    Sophus::SE3d a_pose_b;
    ProtoToSophus(pose.a_pose_b(), &a_pose_b);

    LOG_EVERY_N(INFO, 200) << pose.ShortDebugString() << " q: "
                           << a_pose_b.unit_quaternion().vec().transpose()
                           << " t: " << a_pose_b.translation().transpose();
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
};

}  // namespace farm_ng

int main(int argc, char* argv[]) {
  // Initialize Google's logging library.
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging(argv[0]);

  try {
    boost::asio::io_service io_service;
    farm_ng::ExtrinsicCalibrator calibrator(io_service);
    io_service.run();
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
