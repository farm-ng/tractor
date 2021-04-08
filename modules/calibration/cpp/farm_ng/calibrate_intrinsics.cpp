#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/calibration/calibrate_intrinsics.pb.h"
#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/calibration/intrinsic_calibrator.h"
#include "farm_ng/calibration/intrinsic_model.pb.h"

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/create_video_dataset.pb.h"
#include "farm_ng/perception/time_series.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(video_dataset, "",
              "The path to a serialized CreateVideoDatasetResult");

DEFINE_string(camera_name, "", "Camera frame name.");

DEFINE_bool(filter_stable_tags, false, "Run filter for stable tags.");
DEFINE_bool(disable_reprojection_images, false,
            "Disable output of reprojection images.");
DEFINE_int32(steady_count, 0, "Number of frames to hold steady.");
DEFINE_int32(steady_window_size, 0,
             "Size of window for stable tag detection in pixels.");

DEFINE_int32(novel_window_size, 0, "Size of window to determine if detection is novel.");

typedef farm_ng::core::Event EventPb;
using farm_ng::core::ArchiveProtobufAsBinaryResource;
using farm_ng::core::ArchiveProtobufAsJsonResource;
using farm_ng::core::BUCKET_INTRINSIC_MODELS;
using farm_ng::core::ContentTypeProtobufJson;
using farm_ng::core::EventBus;
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::ReadProtobufFromResource;
using farm_ng::core::SetArchivePath;
using farm_ng::core::Subscription;
using farm_ng::perception::ApriltagDetections;
using farm_ng::perception::CreateVideoDatasetResult;

namespace farm_ng {
namespace calibration {

class CalibrateIntrinsicsProgram {
 public:
  CalibrateIntrinsicsProgram(EventBus& bus,
                             CalibrateIntrinsicsConfiguration configuration,
                             bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({"^" + bus_.GetName() + "/"});
    bus_.GetEventSignal()->connect(std::bind(
        &CalibrateIntrinsicsProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }
    LOG(INFO) << "config:\n" << configuration_.DebugString();
    CalibrateIntrinsicsResult result;

    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    auto dataset_result = ReadProtobufFromResource<CreateVideoDatasetResult>(
        configuration_.video_dataset());
    LOG(INFO) << "dataset_result:\n" << dataset_result.DebugString();

    auto output_dir =
        boost::filesystem::path(dataset_result.dataset().path()).parent_path();

    // Output under the same directory as the dataset.
    SetArchivePath((output_dir / "intrinsic_model").string());

    IntrinsicModel solved_model_pb =
        SolveIntrinsicsModel(InitialIntrinsicModelFromConfig(configuration_));

    if (solved_model_pb.solver_status() ==
        SolverStatus::SOLVER_STATUS_CONVERGED) {
      ModelError(&solved_model_pb, FLAGS_disable_reprojection_images);
    }

    LOG(INFO) << "Initial model computed.";

    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_intrinsics_solved()->CopyFrom(
        ArchiveProtobufAsBinaryResource("solved", solved_model_pb));
    result.mutable_camera_model()->CopyFrom(ArchiveProtobufAsJsonResource(
        "camera_model", solved_model_pb.camera_model()));
    result.set_rmse(solved_model_pb.rmse());
    result.set_solver_status(solved_model_pb.solver_status());
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());

    ArchiveProtobufAsJsonResource(configuration_.camera_name(), result);
    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        BUCKET_INTRINSIC_MODELS, configuration_.camera_name(), result));
    LOG(INFO) << "status:\n"
              << status_.DebugString() << "\nresult:\n"
              << result.DebugString();

    send_status();
    return 0;
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
    timer_.async_wait(std::bind(&CalibrateIntrinsicsProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    CalibrateIntrinsicsConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CalibrateIntrinsicsConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    CHECK(event.name().rfind(bus_.GetName() + "/", 0) == 0);
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CalibrateIntrinsicsConfiguration configuration_;
  CalibrateIntrinsicsStatus status_;
  CalibrateIntrinsicsResult result_;
};

}  // namespace calibration
}  // namespace farm_ng

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup."; }

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::CalibrateIntrinsicsConfiguration config;

  config.mutable_video_dataset()->set_path(FLAGS_video_dataset);
  config.mutable_video_dataset()->set_content_type(
      ContentTypeProtobufJson<CreateVideoDatasetResult>());
  config.set_camera_name(FLAGS_camera_name);
  config.set_filter_stable_tags(FLAGS_filter_stable_tags);
  config.set_distortion_model(farm_ng::perception::CameraModel::DISTORTION_MODEL_PANO_TOOLS_DERSCH);
  if (FLAGS_steady_window_size > 0) {
    config.mutable_steady_window_size()->set_value(FLAGS_steady_window_size);
  }
  if (FLAGS_steady_count > 0) {
    config.mutable_steady_count()->set_value(FLAGS_steady_count);
  }

  if(FLAGS_novel_window_size > 0) {
config.mutable_novel_window_size()->set_value(FLAGS_novel_window_size);
  }
  farm_ng::calibration::CalibrateIntrinsicsProgram program(bus, config,
                                                           FLAGS_interactive);
  return program.run();
}
int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
