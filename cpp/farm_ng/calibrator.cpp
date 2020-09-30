#include "farm_ng/ipc.h"

#include <future>
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>

#include "farm_ng/calibration/apriltag_rig_calibrator.h"

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
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::LoggingCommand;
using farm_ng_proto::tractor::v1::LoggingStatus;
using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::SolverStatus;

using Sophus::SE3d;
using Sophus::Vector6d;

namespace farm_ng {
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
