#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ceres/ceres.h>
#include <opencv2/imgproc.hpp>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include "farm_ng/calibration/robot_hal.grpc.pb.h"

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/calibration/camera_rig_apriltag_rig_cost_functor.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"
#include "farm_ng/calibration/robot_hal_client.h"

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
DEFINE_string(calibration, "", "RobotArmExtrinsicsModel model");
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

std::vector<CapturePoseResponse> CapturePoseResponses(
    const CaptureRobotExtrinsicsDatasetResult& dataset_result) {
  core::EventLogReader log_reader(dataset_result.dataset());
  std::vector<CapturePoseResponse> responses;
  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (const std::runtime_error& e) {
      break;
    }

    CapturePoseResponse pose_response;
    if (event.data().UnpackTo(&pose_response)) {
      VLOG(2) << "Response:\n" << pose_response.ShortDebugString();
      for (auto& image : *pose_response.mutable_images()) {
        if (image.resource().payload_case() ==
            core::Resource::PayloadCase::kPath) {
          std::ifstream bin_in(
              (core::GetBlobstoreRoot() / image.resource().path()).string(),
              std::ifstream::binary);
          CHECK(bin_in.good());
          std::string bin_str((std::istreambuf_iterator<char>(bin_in)),
                              std::istreambuf_iterator<char>());
          image.mutable_resource()->set_data(bin_str);
        }
      }
      responses.push_back(pose_response);
    }
  }
  return responses;
}

class RobotHalServiceMock final : public RobotHALService::Service {
 public:
  RobotHalServiceMock(core::EventBus& bus, bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      // status_.mutable_input_required_resource()->CopyFrom(resource);
    } else {
      // set_configuration(resource);
    }
    bus_.AddSubscriptions({bus_.GetName()});
    bus_.GetEventSignal()->connect(
        std::bind(&RobotHalServiceMock::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  grpc::Status CapturePose(
      grpc::ServerContext* context,
      grpc::ServerReaderWriter<CapturePoseResponse, CapturePoseRequest>* stream)
      override {
    return grpc::Status::OK;
  }

  grpc::Status CalibrationResult(grpc::ServerContext* context,
                                 const CalibrationResultRequest* request,
                                 CalibrationResultResponse* response) override {
    return grpc::Status::OK;
  }

  grpc::Status CalibratedCapture(grpc::ServerContext* context,
                                 const CalibratedCaptureRequest* request,
                                 CalibratedCaptureResponse* response) override {
    response->set_capture_id(index_);
    response->mutable_robot_arm()->CopyFrom(
        dataset_result_.configuration().robot_arm());
    response->mutable_camera_rig()->CopyFrom(
        calibration_.base_camera_rig_model().camera_rig());
    auto& r = responses_[index_];
    response->mutable_workspace_poses()->CopyFrom(
        calibration_.workspace_poses());
    response->mutable_robot_poses()->CopyFrom(r.poses());
    response->mutable_joint_states()->CopyFrom(r.joint_states());
    response->mutable_images()->CopyFrom(r.images());
    response->mutable_stamp()->CopyFrom(core::MakeTimestampNow());

    index_ = (index_ + 1) % responses_.size();
    return grpc::Status::OK;
  }

  grpc::Status ApriltagRigPoseEstimate(
      grpc::ServerContext* context,
      const ApriltagRigPoseEstimateRequest* request,
      ApriltagRigPoseEstimateResponse* response) override {
    LOG(INFO) << request->ShortDebugString();
    return grpc::Status::OK;
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
    timer_.async_wait(
        std::bind(&RobotHalServiceMock::on_timer, this, std::placeholders::_1));

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
    dataset_result_ =
        core::ReadProtobufFromResource<CaptureRobotExtrinsicsDatasetResult>(
            dataset_resource);

    calibration_ = core::ReadProtobufFromJsonFile<RobotArmExtrinsicsModel>(
        FLAGS_calibration);
    responses_ = CapturePoseResponses(dataset_result_);

    auto output_dir =
        boost::filesystem::path(dataset_result_.dataset().path()).parent_path();

    // Output under the same directory as the dataset.
    core::SetArchivePath((output_dir / "mock").string());

    std::string server_address("0.0.0.0:50051");

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(this);
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;
    bus_.get_io_service().run();
    server->Shutdown();
    server->Wait();
    return 0;
  }

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CaptureRobotExtrinsicsDatasetResult dataset_result_;
  RobotArmExtrinsicsModel calibration_;

  std::vector<CapturePoseResponse> responses_;
  size_t index_ = 0;
};

}  // namespace farm_ng::calibration

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::RobotHalServiceMock program(bus, FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup"; }

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
