#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/algorithm/string.hpp>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_pipeline_utils.h"

#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/create_video_dataset.pb.h"
#include "farm_ng/perception/image.pb.h"

DEFINE_bool(interactive, false, "receive program args via eventbus");
DEFINE_string(name, "default",
              "a dataset name, used in the output archive name");
DEFINE_string(video_path, "", "A video file path.");
DEFINE_string(camera_frame_name, "camera01",
              "Frame name to use for the camera model.");

DEFINE_string(apriltag_rigs,
              "calibration_boards/board_430_506.json,"
              "calibration_boards/board_507_583.json",
              "A comma seperated list of blobstore relative paths of apriltag "
              "rigs to detect.");

DEFINE_bool(detect_apriltags, true, "Detect apriltags.");

typedef farm_ng::core::Event EventPb;
using farm_ng::core::ArchiveProtobufAsJsonResource;
using farm_ng::core::BUCKET_VIDEO_DATASETS;
using farm_ng::core::EventBus;
using farm_ng::core::EventLogWriter;
using farm_ng::core::GetUniqueArchiveResource;
using farm_ng::core::LoggingStatus;
using farm_ng::core::MakeEvent;
using farm_ng::core::MakeTimestampNow;
using farm_ng::core::Subscription;

namespace farm_ng {
namespace perception {

class CreateVideoDatasetProgram {
 public:
  CreateVideoDatasetProgram(EventBus& bus,
                            CreateVideoDatasetConfiguration configuration,
                            bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }
    bus_.AddSubscriptions({bus_.GetName()});

    bus_.GetEventSignal()->connect(std::bind(
        &CreateVideoDatasetProgram::on_event, this, std::placeholders::_1));
    on_timer(boost::system::error_code());
  }

  int run() {
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    CreateVideoDatasetResult result;

    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    std::vector<ApriltagRig> rigs;
    ApriltagConfig apriltag_config;
    for (auto rig_resource : configuration_.apriltag_rigs()) {
      ApriltagRig rig =
          core::ReadProtobufFromResource<ApriltagRig>(rig_resource);
      rigs.push_back(rig);
      for (const auto& node : rig.nodes()) {
        auto* tag = apriltag_config.mutable_tag_library()->add_tags();
        tag->set_id(node.id());
        tag->set_size(node.tag_size());
      }
    }

    core::SetArchivePath(
        (core::GetBucketRelativePath(core::BUCKET_LOGS) / configuration_.name())
            .string());
    auto resource_path = GetUniqueArchiveResource("events", "log", "");

    core::EventLogWriter log_writer(resource_path.second);
    CameraPipelineCommand tracking_camera_command;

    CHECK_EQ(1, configuration_.video_file_cameras_size());
    cv::VideoCapture capture(
        (core::GetBlobstoreRoot() /
         configuration_.video_file_cameras(0).video_file_resource().path())
            .string());

    std::optional<CameraModel> camera_model;

    std::optional<ApriltagDetector> detector;

    Image image_pb;
    image_pb.mutable_fps()->set_value(30);
    image_pb.mutable_frame_number()->set_value(0);
    image_pb.mutable_resource()->CopyFrom(
        configuration_.video_file_cameras(0).video_file_resource());

    while (true) {
      cv::Mat image;
      capture >> image;
      if (image.empty()) {
        break;
      }
      if (!camera_model) {
        camera_model = DefaultCameraModel(
            configuration_.video_file_cameras(0).camera_frame_name(),
            image.size().width, image.size().height);
        detector = ApriltagDetector(*camera_model, nullptr, &apriltag_config);
        image_pb.mutable_camera_model()->CopyFrom(*camera_model);
      }

      CHECK_EQ(camera_model->image_width(), image.size().width);
      CHECK_EQ(camera_model->image_height(), image.size().height);

      cv::Mat gray;
      if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      } else {
        CHECK_EQ(1, image.channels())
            << "Image should be grayscale or BGR, something is wrong...";
        gray = image.clone();
      }
      auto stamp = core::MakeTimestampNow();
      log_writer.Write(
          MakeEvent(camera_model->frame_name() + "/image", image_pb, stamp));

      bool first_frame_for_camera = true;
      for (auto& entry : *status_.mutable_per_camera_num_frames()) {
        if (entry.camera_name() == image_pb.camera_model().frame_name()) {
          entry.set_num_frames(entry.num_frames() + 1);
          first_frame_for_camera = false;
        }
      }
      if (first_frame_for_camera) {
        auto per_camera_num_frames = status_.add_per_camera_num_frames();
        per_camera_num_frames->set_camera_name(
            image_pb.camera_model().frame_name());
        per_camera_num_frames->set_num_frames(1);
      }

      if (configuration_.detect_apriltags()) {
        auto detections = detector->Detect(gray, stamp);
        detections.mutable_image()->CopyFrom(image_pb);
        log_writer.Write(MakeEvent(camera_model->frame_name() + "/apriltags",
                                   detections, stamp));

        for (auto const& detection : detections.detections()) {
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

      image_pb.mutable_frame_number()->set_value(
          image_pb.frame_number().value() + 1);
    }

    bus_.get_io_service().poll();

    result.mutable_configuration()->CopyFrom(configuration_);
    result.mutable_per_camera_num_frames()->CopyFrom(
        status_.per_camera_num_frames());
    result.mutable_per_tag_id_num_frames()->CopyFrom(
        status_.per_tag_id_num_frames());

    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    result.mutable_dataset()->set_path(resource_path.first.path());

    // TODO some how save the result in the archive directory as well, so its
    // self contained.
    ArchiveProtobufAsJsonResource(configuration_.name(), result);

    status_.mutable_result()->CopyFrom(WriteProtobufAsJsonResource(
        BUCKET_VIDEO_DATASETS, configuration_.name(), result));
    LOG(INFO) << "Complete:\n" << status_.DebugString();
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
    timer_.async_wait(std::bind(&CreateVideoDatasetProgram::on_timer, this,
                                std::placeholders::_1));

    send_status();
  }

  bool on_configuration(const EventPb& event) {
    CreateVideoDatasetConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  void set_configuration(CreateVideoDatasetConfiguration configuration) {
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
  CreateVideoDatasetConfiguration configuration_;
  CreateVideoDatasetStatus status_;
};

}  // namespace perception
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::perception::CreateVideoDatasetConfiguration config;
  config.set_name(FLAGS_name);
  config.set_detect_apriltags(FLAGS_detect_apriltags);
  auto* video_file_camera = config.add_video_file_cameras();

  video_file_camera->mutable_video_file_resource()->set_path(FLAGS_video_path);
  video_file_camera->mutable_video_file_resource()->set_content_type(
      "video/mp4");
  video_file_camera->set_camera_frame_name(FLAGS_camera_frame_name);

  std::stringstream ss(FLAGS_apriltag_rigs);
  std::string token;
  while (std::getline(ss, token, ',')) {
    auto* resource = config.add_apriltag_rigs();
    boost::trim(token);
    resource->set_path(token);
    resource->set_content_type(farm_ng::core::ContentTypeProtobufJson<
                               farm_ng::perception::ApriltagRig>());
  }

  farm_ng::perception::CreateVideoDatasetProgram program(bus, config,
                                                         FLAGS_interactive);
  return program.run();
}

void Cleanup(farm_ng::core::EventBus& bus) {}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
