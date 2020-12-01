#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/calibrate_apriltag_rig.pb.h"
#include "farm_ng/calibration/calibrator.pb.h"
#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/capture_video_dataset.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(calibration_dataset, "",
              "The path to a serialized CaptureVideoDatasetResult");

DEFINE_string(tag_ids, "", "List of tag ids, comma separated list of ints.");
DEFINE_string(name, "rig", "Name of the rig.");

DEFINE_int32(
    root_tag_id, -1,
    "The root tag id, -1 will result in root_tag_id == first value in tag_ids");

DEFINE_bool(filter_stable_tags, false, "Run filter for stable tags.");
DEFINE_string(camera_name, "camera_pipeline/front/left",
              "Which camera to run on.");

typedef farm_ng::core::Event EventPb;
using farm_ng::core::ArchiveProtobufAsBinaryResource;
using farm_ng::core::ArchiveProtobufAsJsonResource;
using farm_ng::core::BUCKET_APRILTAG_RIG_MODELS;
using farm_ng::core::ContentTypeProtobufJson;
using farm_ng::core::EventBus;
using farm_ng::core::EventLogReader;
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::ReadProtobufFromResource;
using farm_ng::core::SetArchivePath;
using farm_ng::core::Subscription;
using farm_ng::perception::ApriltagDetections;
using farm_ng::perception::ApriltagsFilter;
using farm_ng::perception::CaptureVideoDatasetResult;

namespace farm_ng {
namespace calibration {

class CalibrateApriltagRigProgram {
 public:
  CalibrateApriltagRigProgram(EventBus& bus,
                              CalibrateApriltagRigConfiguration configuration,
                              bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({"^" + bus_.GetName() + "/"});
    bus_.GetEventSignal()->connect(std::bind(
        &CalibrateApriltagRigProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  bool is_calibration_event(const std::string& s) {
    return s.rfind(configuration_.camera_name() + "/apriltags", 0) == 0;
  }

  void OnLogEvent(const EventPb& event, ApriltagRigCalibrator* calibrator) {
    if (!is_calibration_event(event.name())) {
      return;
    }
    ApriltagDetections detections;
    if (!event.data().UnpackTo(&detections)) {
      return;
    }
    bool add_frame = true;
    if (configuration_.filter_stable_tags()) {
      add_frame = tag_filter_.AddApriltags(detections);
    }
    if (add_frame) {
      calibrator->AddFrame(detections);
    }
  }

  // reads the event log from the CalibrationDatasetResult, and
  // populates a MonocularApriltagRigModel to be solved.
  template <typename DatasetType>
  ApriltagRigModel LoadCalibrationDataset(const DatasetType& dataset_result) {
    EventLogReader log_reader(dataset_result.dataset());
    ApriltagRigCalibrator calibrator(configuration_);
    while (true) {
      EventPb event;
      try {
        bus_.get_io_service().poll();
        event = log_reader.ReadNext();
        OnLogEvent(event, &calibrator);
      } catch (std::runtime_error& e) {
        break;
      }
    }
    return calibrator.PoseInitialization();
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }
    LOG(INFO) << "config:\n" << configuration_.DebugString();

    auto dataset_result = ReadProtobufFromResource<CaptureVideoDatasetResult>(
        configuration_.calibration_dataset());
    LOG(INFO) << "dataset_result:\n" << dataset_result.DebugString();

    auto output_dir =
        boost::filesystem::path(dataset_result.dataset().path()).parent_path();

    // Output under the same directory as the dataset.
    SetArchivePath(
        (output_dir / "apriltag_rig_model" / configuration_.camera_name())
            .string());
    ApriltagRigModel model = LoadCalibrationDataset(dataset_result);
    LOG(INFO) << "Initial model computed.";

    MonocularApriltagRigModel initial_model_pb;
    model.ToMonocularApriltagRigModel(&initial_model_pb);

    CalibrateApriltagRigResult result;
    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_monocular_apriltag_rig_initial()->CopyFrom(
        ArchiveProtobufAsBinaryResource("initial", initial_model_pb));
    result.set_rmse(model.rmse);
    result.set_solver_status(model.status);
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    // if (log.has_recording()) {
    //      result.mutable_event_log()->set_path(log.recording().archive_path());
    //}
    status_.mutable_result()->CopyFrom(
        ArchiveProtobufAsJsonResource("initial_result", result));
    send_status();

    Solve(&model);
    MonocularApriltagRigModel final_model_pb;
    model.ToMonocularApriltagRigModel(&final_model_pb);
    result.mutable_monocular_apriltag_rig_solved()->CopyFrom(
        ArchiveProtobufAsBinaryResource("solved", final_model_pb));
    result.set_rmse(model.rmse);
    result.set_solver_status(model.status);
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());

    // TODO some how save the result in the archive directory as well, so its
    // self contained.
    ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        BUCKET_APRILTAG_RIG_MODELS, configuration_.name(), result));

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
    timer_.async_wait(std::bind(&CalibrateApriltagRigProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    CalibrateApriltagRigConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CalibrateApriltagRigConfiguration configuration) {
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
  CalibrateApriltagRigConfiguration configuration_;
  CalibrateApriltagRigStatus status_;
  CalibrateApriltagRigResult result_;

  ApriltagsFilter tag_filter_;
};

}  // namespace calibration
}  // namespace farm_ng

void Cleanup(farm_ng::core::EventBus& bus) { LOG(INFO) << "Cleanup."; }

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::calibration::CalibrateApriltagRigConfiguration config;
  std::stringstream ss(FLAGS_tag_ids);
  std::string token;
  while (std::getline(ss, token, ',')) {
    config.add_tag_ids(stoi(token));
  }
  config.mutable_calibration_dataset()->set_path(FLAGS_calibration_dataset);
  config.mutable_calibration_dataset()->set_content_type(
      ContentTypeProtobufJson<CaptureVideoDatasetResult>());
  config.set_root_tag_id(FLAGS_root_tag_id);
  config.set_name(FLAGS_name);
  config.set_filter_stable_tags(FLAGS_filter_stable_tags);
  config.set_camera_name(FLAGS_camera_name);

  farm_ng::calibration::CalibrateApriltagRigProgram program(bus, config,
                                                            FLAGS_interactive);
  return program.run();
}
int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
