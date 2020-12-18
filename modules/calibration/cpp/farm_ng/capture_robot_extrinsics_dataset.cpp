#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>
#include <opencv2/imgcodecs.hpp>
#include <sophus/se3.hpp>

#include "farm_ng/calibration/capture_robot_extrinsics_dataset.pb.h"
#include "farm_ng/calibration/robot_hal.grpc.pb.h"
#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");

DEFINE_string(configuration_path,
              "configurations/capture_robot_extrinsics_dataset.json",
              "Blobstore-relative path to the configuration file.");

using farm_ng::core::MakeEvent;
using farm_ng::core::ReadProtobufFromJsonFile;
using farm_ng::perception::Image;
using farm_ng::perception::NamedSE3Pose;
typedef farm_ng::core::Event EventPb;

namespace farm_ng::calibration {
class RobotHalClient {
 public:
  RobotHalClient(std::shared_ptr<grpc::Channel> channel)
      : stub_(RobotHALService::NewStub(channel)) {}

  std::tuple<grpc::Status, CapturePoseResponse> CapturePoseSync(
      const CapturePoseRequest& request) {
    grpc::ClientContext context;

    std::shared_ptr<
        grpc::ClientReaderWriter<CapturePoseRequest, CapturePoseResponse> >
        stream(stub_->CapturePose(&context));

    stream->Write(request);
    stream->WritesDone();

    // Block waiting for response
    CapturePoseResponse response;
    stream->Read(&response);
    return {stream->Finish(), response};
  }

 private:
  std::unique_ptr<RobotHALService::Stub> stub_;
};

std::vector<CapturePoseRequest> GenerateCapturePoseRequests(
    const CaptureRobotExtrinsicsDatasetConfiguration& configuration) {
  std::vector<CapturePoseRequest> pose_requests;
  for(auto base_pose_link: configuration.base_poses_link()) {
      CapturePoseRequest pose_request;
        NamedSE3Pose* pose = pose_request.add_poses();
        pose->CopyFrom(base_pose_link);
        pose_requests.push_back(pose_request);
  }
  return pose_requests;
}

void ImageDataToResource(Image* image, int frame_number) {
  core::Resource* resource = image->mutable_resource();
  CHECK(resource->payload_case() == core::Resource::kData);
  cv::Mat mat =
      cv::imdecode(cv::Mat(1, resource->data().size(), CV_8UC1,
                           const_cast<char*>(resource->data().data())),
                   cv::IMREAD_UNCHANGED);

  auto resource_path = core::GetUniqueArchiveResource(
      perception::FrameNameNumber(image->camera_model().frame_name(),
                                  frame_number),
      "png", "image/png");
  resource->CopyFrom(resource_path.first);
  CHECK(resource->payload_case() == core::Resource::kPath);
  LOG(INFO) << resource_path.second.string();
  CHECK(cv::imwrite(resource_path.second.string(), mat))
      << "Could not write: " << resource_path.second;
}

class CaptureRobotExtrinsicsDatasetProgram {
 public:
  CaptureRobotExtrinsicsDatasetProgram(
      core::EventBus& bus,
      const CaptureRobotExtrinsicsDatasetConfiguration& configuration)
      : bus_(bus),
        timer_(bus_.get_io_service()),
        configuration_(configuration) {
    if (FLAGS_interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName()});

    bus_.GetEventSignal()->connect(
        std::bind(&CaptureRobotExtrinsicsDatasetProgram::on_event, this,
                  std::placeholders::_1));
    on_timer(boost::system::error_code());
  }
  void send_status() {
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&CaptureRobotExtrinsicsDatasetProgram::on_timer,
                                this, std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    CaptureRobotExtrinsicsDatasetConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(
      const CaptureRobotExtrinsicsDatasetConfiguration& configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    status_.mutable_configuration()->CopyFrom(configuration_);
    send_status();
  }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {});

    grpc::ChannelArguments ch_args;
    ch_args.SetMaxReceiveMessageSize(100000000);
    ch_args.SetMaxSendMessageSize(100000000);
    auto credentials = grpc::InsecureChannelCredentials();
    auto channel = grpc::CreateCustomChannel(
        configuration_.hal_service_address(), credentials, ch_args);

    farm_ng::calibration::RobotHalClient client(channel);

    std::string log_path = (core::GetBucketRelativePath(core::BUCKET_LOGS) /
                            boost::filesystem::path(configuration_.name()))
                               .string();

    core::SetArchivePath(log_path);

    auto resource_path = farm_ng::core::GetUniqueArchiveResource(
        "events", "log", "application/farm_ng.eventlog.v1");

    core::EventLogWriter log_writer(resource_path.second);

    auto requests = GenerateCapturePoseRequests(configuration_);
    status_.clear_request_queue();
    for (auto& request : requests) {
      status_.add_request_queue()->CopyFrom(request);
    }
    int frame_number = 0;
    for (auto& request : requests) {
      bus_.get_io_service().poll();

      log_writer.Write(core::MakeEvent("capture/request", request));

      auto [status, response] = client.CapturePoseSync(request);

      CHECK(status.ok()) << status.error_message();

      CHECK_EQ(response.status(), CapturePoseResponse::STATUS_SUCCESS);
      for (const Image& image : response.images()) {
        CHECK_GT(image.resource().data().length(), 0);
        CHECK_GT(image.camera_model().image_width(), 0);
      }

      for (Image& image : *response.mutable_images()) {
        ImageDataToResource(&image, frame_number);
      }
      log_writer.Write(core::MakeEvent("capture/response", response));
      status_.set_latest_request_index(frame_number);
      status_.mutable_latest_response()->CopyFrom(response);
      send_status();

      frame_number++;
    }
    CaptureRobotExtrinsicsDatasetResult result;
    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_dataset()->CopyFrom(resource_path.first);

    core::ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        core::BUCKET_ROBOT_EXTRINSICS_DATASETS, configuration_.name(), result));

    return 0;
  }

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;

  CaptureRobotExtrinsicsDatasetConfiguration configuration_;
  CaptureRobotExtrinsicsDatasetStatus status_;
};

}  // namespace farm_ng::calibration

int Main(farm_ng::core::EventBus& bus) {
  auto configuration = ReadProtobufFromJsonFile<
      farm_ng::calibration::CaptureRobotExtrinsicsDatasetConfiguration>(
      farm_ng::core::GetBlobstoreRoot() / FLAGS_configuration_path);

  farm_ng::calibration::CaptureRobotExtrinsicsDatasetProgram program(
      bus, configuration);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup"; }

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
