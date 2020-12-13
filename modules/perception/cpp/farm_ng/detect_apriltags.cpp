#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include <boost/algorithm/string.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/capture_video_dataset.pb.h"
#include "farm_ng/perception/detect_apriltags.pb.h"
#include "farm_ng/perception/image.pb.h"
#include "farm_ng/perception/image_loader.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");
DEFINE_string(video_dataset, "",
              "A video dataset, a CaptureVideoDatasetResult json file.");
DEFINE_string(tag_config, "",
              "ApriltagConfig json path, relative to blobstore.  If not set, "
              "uses configurations/apriltag.json");

typedef farm_ng::core::Event EventPb;
using farm_ng::core::ArchiveProtobufAsJsonResource;
using farm_ng::core::BUCKET_VIDEO_DATASETS;
using farm_ng::core::ContentTypeProtobufJson;
using farm_ng::core::EventBus;
using farm_ng::core::EventLogReader;
using farm_ng::core::LoggingStatus;
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::Subscription;

namespace farm_ng {
namespace perception {

class DetectApriltagsProgram {
 public:
  DetectApriltagsProgram(EventBus& bus,
                         DetectApriltagsConfiguration configuration,
                         bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName(), "logger/command", "logger/status"});

    bus_.GetEventSignal()->connect(std::bind(&DetectApriltagsProgram::on_event,
                                             this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    WaitForServices(bus_, {"ipc_logger"});

    CaptureVideoDatasetResult result;

    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    LoggingStatus log = StartLogging(bus_, configuration_.name());

    CaptureVideoDatasetResult video_dataset =
        core::ReadProtobufFromResource<CaptureVideoDatasetResult>(
            configuration_.video_dataset());

    EventLogReader log_reader(video_dataset.dataset());

    std::map<std::string, std::shared_ptr<perception::ApriltagDetector>>
        per_camera_detectors;
    std::optional<perception::ApriltagConfig> tag_config;
    if (configuration_.has_tag_config()) {
      tag_config = core::ReadProtobufFromResource<ApriltagConfig>(
          configuration_.tag_config());
    }
    ImageLoader image_loader;
    while (true) {
      EventPb event;
      try {
        event = log_reader.ReadNext();
      } catch (std::runtime_error& e) {
        break;
      }

      // We'll resend all events, so they're logged by the logger,
      // except for detections, as we're redetecting them.
      if (event.data().Is<ApriltagDetections>()) {
        continue;
      }
      bus_.Send(event);

      Image image;
      ApriltagDetections unfiltered_detections;
      bool has_detections = false;
      if (event.data().UnpackTo(&image)) {
        if (image.camera_model().distortion_coefficients_size() == 0) {
          for (int i = 0; i < 8; ++i) {
            image.mutable_camera_model()->add_distortion_coefficients(0);
          }
        }
        LOG(INFO) << image.camera_model().frame_name() << " "
                  << image.frame_number().ShortDebugString() << " "
                  << image.resource().ShortDebugString();
        if (per_camera_detectors.count(image.camera_model().frame_name()) ==
            0) {
          const perception::ApriltagConfig* tag_config_ptr = nullptr;
          if (tag_config.has_value()) {
            tag_config_ptr = &tag_config.value();
          }
          per_camera_detectors.emplace(
              image.camera_model().frame_name(),
              std::shared_ptr<perception::ApriltagDetector>(
                  new perception::ApriltagDetector(image.camera_model(),
                                                   nullptr, tag_config_ptr)));
        }
        cv::Mat image_mat = image_loader.LoadImage(image);
        cv::Mat gray;
        if (image_mat.channels() == 3) {
          cv::cvtColor(image_mat, gray, cv::COLOR_BGR2GRAY);
        } else {
          CHECK_EQ(image_mat.channels(), 1);
          gray = image_mat;
        }
        unfiltered_detections =
            per_camera_detectors[image.camera_model().frame_name()]->Detect(
                gray, event.stamp());
        unfiltered_detections.mutable_image()->CopyFrom(image);

        auto tag_event =
            core::MakeEvent(image.camera_model().frame_name() + "/apriltags",
                            unfiltered_detections, event.stamp());
        bus_.Send(tag_event);
        bool first_frame_for_camera = true;
        for (auto& entry : *status_.mutable_per_camera_num_frames()) {
          if (entry.camera_name() == image.camera_model().frame_name()) {
            entry.set_num_frames(entry.num_frames() + 1);
            first_frame_for_camera = false;
          }
        }
        if (first_frame_for_camera) {
          auto per_camera_num_frames = status_.add_per_camera_num_frames();
          per_camera_num_frames->set_camera_name(
              image.camera_model().frame_name());
          per_camera_num_frames->set_num_frames(1);
        }

        for (auto const& detection : unfiltered_detections.detections()) {
          bool first_time_seen = true;
          for (auto& entry : *status_.mutable_per_tag_id_num_frames()) {
            if (entry.tag_id() == detection.id()) {
              entry.set_num_frames(entry.num_frames() + 1);
              first_time_seen = false;
            }
          }
          if (first_time_seen) {
            auto per_tag_id_num_frames = status_.add_per_tag_id_num_frames();
            per_tag_id_num_frames->set_tag_id(detection.id());
            per_tag_id_num_frames->set_num_frames(1);
          }
        }
      }
      bus_.get_io_service().poll();
    }

    bus_.get_io_service().poll();

    result.mutable_per_camera_num_frames()->CopyFrom(
        status_.per_camera_num_frames());
    result.mutable_per_tag_id_num_frames()->CopyFrom(
        status_.per_tag_id_num_frames());

    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    result.mutable_dataset()->set_path(log.recording().archive_path());

    ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        BUCKET_VIDEO_DATASETS, configuration_.name(), result));
    LOG(INFO) << "Complete:\n" << status_.DebugString();
    send_status();
    return 0;
  }

  void send_status() {
    LOG(INFO) << status_.ShortDebugString();
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&DetectApriltagsProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    DetectApriltagsConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(DetectApriltagsConfiguration configuration) {
    configuration_ = configuration;
    status_.clear_input_required_configuration();
    send_status();
  }

  void on_event(const EventPb& event) {
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
  DetectApriltagsConfiguration configuration_;
  DetectApriltagsStatus status_;
};

}  // namespace perception
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::perception::DetectApriltagsConfiguration config;
  config.set_name(FLAGS_name);
  config.mutable_video_dataset()->set_path(FLAGS_video_dataset);
  config.mutable_video_dataset()->set_content_type(
      ContentTypeProtobufJson<
          farm_ng::perception::CaptureVideoDatasetResult>());

  if (!FLAGS_tag_config.empty()) {
    config.mutable_tag_config()->set_path(FLAGS_tag_config);
    config.mutable_tag_config()->set_content_type(
        ContentTypeProtobufJson<farm_ng::perception::ApriltagConfig>());
  }

  farm_ng::perception::DetectApriltagsProgram program(bus, config,
                                                      FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) {
  farm_ng::core::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
