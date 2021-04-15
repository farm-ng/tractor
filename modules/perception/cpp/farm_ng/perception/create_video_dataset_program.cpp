#include <optional>

#include <glog/logging.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <google/protobuf/util/time_util.h>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/event_log.h"
#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/camera_pipeline_utils.h"

#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/create_video_dataset.pb.h"
#include "farm_ng/perception/create_video_dataset_program.h"
#include "farm_ng/perception/image.pb.h"

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

CreateVideoDatasetProgram::CreateVideoDatasetProgram(
    EventBus& bus, CreateVideoDatasetConfiguration configuration,
    bool interactive)
    : bus_(bus), timer_(bus_.get_io_service()) {
  if (interactive) {
    status_.mutable_input_required_configuration()->CopyFrom(configuration);
  } else {
    set_configuration(configuration);
  }
  bus_.AddSubscriptions({bus_.GetName()});

  bus_.GetEventSignal()->connect(std::bind(&CreateVideoDatasetProgram::on_event,
                                           this, std::placeholders::_1));
  on_timer(boost::system::error_code());
}

int CreateVideoDatasetProgram::run() {
  while (status_.has_input_required_configuration()) {
    bus_.get_io_service().run_one();
  }

  CreateVideoDatasetResult result;

  result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

  std::vector<ApriltagRig> rigs;
  ApriltagConfig apriltag_config;
  for (auto rig_resource : configuration_.apriltag_rigs()) {
    ApriltagRig rig = core::ReadProtobufFromResource<ApriltagRig>(rig_resource);
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


  for (int camera_id = 0; camera_id < configuration_.video_file_cameras_size();
       ++camera_id) {
    std::string video_path = NativePathFromResourcePath( configuration_.video_file_cameras(camera_id)
                                        .video_file_resource()).string();


    CHECK(boost::filesystem::exists(video_path)) << configuration_.video_file_cameras(camera_id).ShortDebugString() << " file does not exist.";

    cv::VideoCapture capture(video_path);
    std::optional<CameraModel> camera_model;

    std::optional<ApriltagDetector> detector;

    Image image_pb;
    image_pb.mutable_fps()->set_value(capture.get(cv::CAP_PROP_FPS));
    image_pb.mutable_frame_number()->set_value(0);
    image_pb.mutable_resource()->CopyFrom(
        configuration_.video_file_cameras(0).video_file_resource());

    while (true) {
      capture.set(cv::CAP_PROP_POS_FRAMES, image_pb.frame_number().value());
      double msec = capture.get(cv::CAP_PROP_POS_MSEC);
      cv::Mat image;
      capture >> image;
      if (image.empty()) {
        break;
      }

      LOG(INFO)
          << configuration_.video_file_cameras(camera_id).camera_frame_name()
          << " Frame: " << image_pb.frame_number().value() << " msec: " << msec;
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

      auto stamp = google::protobuf::util::TimeUtil::NanosecondsToTimestamp(msec*1e9);
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
        double scale = 0.0;
        if (configuration_.has_detection_scale()) {
          scale = configuration_.detection_scale().value();
        }
        auto detections = detector->Detect(gray, stamp, scale);
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
      int next = 1;
      if (configuration_.has_skip_frames()) {
        next = configuration_.skip_frames().value();
      }
      image_pb.mutable_frame_number()->set_value(
          image_pb.frame_number().value() + next);
    }
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

void CreateVideoDatasetProgram::send_status() {
  bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
}

void CreateVideoDatasetProgram::on_timer(
    const boost::system::error_code& error) {
  if (error) {
    LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
    return;
  }
  timer_.expires_from_now(boost::posix_time::millisec(1000));
  timer_.async_wait(std::bind(&CreateVideoDatasetProgram::on_timer, this,
                              std::placeholders::_1));

  send_status();
}

bool CreateVideoDatasetProgram::on_configuration(const EventPb& event) {
  CreateVideoDatasetConfiguration configuration;
  if (!event.data().UnpackTo(&configuration)) {
    return false;
  }
  LOG(INFO) << configuration.ShortDebugString();
  set_configuration(configuration);
  return true;
}

void CreateVideoDatasetProgram::set_configuration(
    CreateVideoDatasetConfiguration configuration) {
  configuration_ = configuration;
  status_.clear_input_required_configuration();
  send_status();
}

void CreateVideoDatasetProgram::on_event(const EventPb& event) {
  if (on_configuration(event)) {
    return;
  }
}

}  // namespace perception
}  // namespace farm_ng
