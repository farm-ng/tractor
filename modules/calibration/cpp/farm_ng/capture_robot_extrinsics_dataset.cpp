#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>
#include <sophus/se3.hpp>

#include "farm_ng/calibration/capture_robot_extrinsics_dataset.pb.h"
#include "farm_ng/calibration/robot_hal_client.h"

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/image_loader.h"
#include "farm_ng/perception/pose_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");

DEFINE_string(configuration_path,
              "configurations/capture_robot_extrinsics_dataset.json",
              "Blobstore-relative path to the configuration file.");

using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::ReadProtobufFromJsonFile;
using farm_ng::perception::Image;
using farm_ng::perception::NamedSE3Pose;
typedef farm_ng::core::Event EventPb;

namespace farm_ng::calibration {

class CaptureRobotExtrinsicsDatasetProgram {
 public:
  CaptureRobotExtrinsicsDatasetProgram(core::EventBus& bus,
                                       core::Resource& resource,
                                       bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_resource()->CopyFrom(resource);
    } else {
      set_configuration(resource);
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
    core::Resource configuration_resource;
    if (!event.data().UnpackTo(&configuration_resource)) {
      return false;
    }
    LOG(INFO) << configuration_resource.ShortDebugString();
    set_configuration(configuration_resource);
    return true;
  }

  void set_configuration(const core::Resource& resource) {
    configuration_ = ReadProtobufFromJsonFile<
        farm_ng::calibration::CaptureRobotExtrinsicsDatasetConfiguration>(
        farm_ng::core::GetBlobstoreRoot() / resource.path());
    status_.clear_input_required_resource();
    status_.mutable_configuration()->CopyFrom(configuration_);
    send_status();
  }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

  int run() {
    while (status_.has_input_required_resource()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {});

    CaptureRobotExtrinsicsDatasetResult result;
    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    farm_ng::calibration::RobotHalClient client(
        configuration_.hal_service_address());

    std::string log_path = (core::GetBucketRelativePath(core::BUCKET_LOGS) /
                            boost::filesystem::path(configuration_.name()))
                               .string();

    core::SetArchivePath(log_path);

    auto resource_path = farm_ng::core::GetUniqueArchiveResource(
        "events", "log", "application/farm_ng.eventlog.v1");

    core::EventLogWriter log_writer(resource_path.second);

    int frame_number = 0;
    for (auto& request : configuration_.request_queue()) {
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
        image.mutable_frame_number()->set_value(frame_number);

        perception::ImageResourceDataToPath(&image);
      }
      auto stamp = core::MakeTimestampNow();
      if (response.has_stamp()) {
        stamp = response.stamp();
      }
      log_writer.Write(core::MakeEvent("capture/response", response, stamp));
      status_.set_latest_request_index(frame_number);
      status_.mutable_latest_response()->CopyFrom(response);
      send_status();

      frame_number++;
    }

    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_dataset()->CopyFrom(resource_path.first);
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());

    core::ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        core::BUCKET_ROBOT_EXTRINSICS_DATASETS, configuration_.name(), result));

    send_status();

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
  farm_ng::core::Resource resource;
  resource.set_path(FLAGS_configuration_path);
  resource.set_content_type(
      farm_ng::core::ContentTypeProtobufJson<
          farm_ng::calibration::CaptureRobotExtrinsicsDatasetConfiguration>());

  farm_ng::calibration::CaptureRobotExtrinsicsDatasetProgram program(
      bus, resource, FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup"; }

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
