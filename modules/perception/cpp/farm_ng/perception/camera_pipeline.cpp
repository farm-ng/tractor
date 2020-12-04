#include "farm_ng/perception/camera_pipeline.h"

#include <opencv2/opencv.hpp>

#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/image_utils.h"

using farm_ng::core::Bucket;
using farm_ng::core::MakeEvent;
using farm_ng::core::ReadProtobufFromJsonFile;

namespace farm_ng {
namespace perception {

typedef boost::signals2::signal<void(
    const std::vector<FrameData>& synced_frames)>
    Signal;

MultiCameraSync::MultiCameraSync(EventBus& event_bus)
    : event_bus_(event_bus), timer_(event_bus.get_io_service()) {}

CameraModel MultiCameraSync::AddCameraConfig(
    const CameraConfig& camera_config) {
  frame_grabbers_.emplace_back(
      FrameGrabber::MakeFrameGrabber(event_bus_, camera_config));
  frame_grabbers_.back()->VisualFrameSignal().connect(
      std::bind(&MultiCameraSync::OnFrame, this, std::placeholders::_1));
  return frame_grabbers_.back()->GetCameraModel();
}

Signal& MultiCameraSync::GetSynchronizedFrameDataSignal() { return signal_; }

std::vector<FrameData> MultiCameraSync::CollectSynchronizedFrameData() {
  std::vector<FrameData> synced_frames;
  std::lock_guard<std::mutex> lock(frame_series_mtx_);
  auto time_window =
      google::protobuf::util::TimeUtil::MillisecondsToDuration(1000.0 / 7);

  for (const auto& grabber : frame_grabbers_) {
    const auto& frame_name = grabber->GetCameraModel().frame_name();
    const auto& series = frame_series_[frame_name];
    auto closest = series.FindNearest(latest_frame_stamp_, time_window);
    if (closest) {
      synced_frames.push_back(*closest);
    } else {
      LOG(INFO) << "Could not find nearest frame for camera: " << frame_name
                << " " << latest_frame_stamp_.ShortDebugString();
    }
  }
  return synced_frames;
}

void MultiCameraSync::GetSynchronizedFrameData(
    const boost::system::error_code& error) {
  if (error) {
    LOG(WARNING) << "Synchronized frame timer error (frame sync may not be "
                    "keeping up): "
                 << error;
    return;
  }
  signal_(CollectSynchronizedFrameData());
}

void MultiCameraSync::OnFrame(const FrameData& frame_data) {
  std::lock_guard<std::mutex> lock(frame_series_mtx_);
  TimeSeries<FrameData>& series =
      frame_series_[frame_data.camera_model.frame_name()];
  series.insert(frame_data);
  series.RemoveBefore(
      frame_data.stamp() -
      google::protobuf::util::TimeUtil::MillisecondsToDuration(500));
  VLOG(2) << frame_data.camera_model.frame_name()
          << " n frames: " << series.size();

  if (frame_data.camera_model.frame_name() ==
      frame_grabbers_.front()->GetCameraModel().frame_name()) {
    latest_frame_stamp_ = frame_data.stamp();
    // schedule a bit into the future to give opportunity for other streams
    // that are slightly offset to be grabbed.
    // TODO base this on frame frate from frame grabber.
    timer_.expires_from_now(std::chrono::milliseconds(int(250.0 / 30.0)));
    timer_.async_wait(std::bind(&MultiCameraSync::GetSynchronizedFrameData,
                                this, std::placeholders::_1));
  }
}

SingleCameraPipeline::SingleCameraPipeline(
    EventBus& event_bus, boost::asio::io_service& pool_service,
    const CameraConfig& camera_config, const CameraModel& camera_model)
    : event_bus_(event_bus),
      strand_(pool_service),
      camera_model_(camera_model),
      detector_(camera_model_),
      video_file_writer_(event_bus_, camera_model_,
                         VideoStreamer::Mode::MODE_MP4_FILE),
      post_count_(0) {
  if (camera_config.has_udp_stream_port()) {
    udp_streamer_ = std::make_unique<VideoStreamer>(
        event_bus_, camera_model_, VideoStreamer::MODE_MP4_UDP,
        camera_config.udp_stream_port().value());
  }
}

void SingleCameraPipeline::Post(CameraPipelineCommand command) {
  strand_.post([this, command] {
    latest_command_.CopyFrom(command);
    LOG(INFO) << "Camera pipeline: " << camera_model_.frame_name()
              << " command: " << latest_command_.ShortDebugString();
  });
}

int SingleCameraPipeline::PostCount() const { return post_count_; }

void SingleCameraPipeline::Post(FrameData frame_data) {
  post_count_++;
  strand_.post([this, frame_data] {
    Compute(frame_data);
    post_count_--;
    CHECK_GE(post_count_, 0);
  });
}

void SingleCameraPipeline::Compute(FrameData frame_data) {
  if (udp_streamer_) {
    udp_streamer_->AddFrame(frame_data.image, frame_data.stamp());
  }
  switch (latest_command_.record_start().mode()) {
    case CameraPipelineCommand::RecordStart::MODE_EVERY_FRAME: {
      video_file_writer_.AddFrame(frame_data.image, frame_data.stamp());
    } break;
    case CameraPipelineCommand::RecordStart::MODE_EVERY_APRILTAG_FRAME: {
      cv::Mat gray;
      if (frame_data.image.channels() != 1) {
        cv::cvtColor(frame_data.image, gray, cv::COLOR_BGR2GRAY);
      } else {
        gray = frame_data.image;
      }
      auto apriltags = detector_.Detect(gray, frame_data.stamp());
      auto image_pb =
          video_file_writer_.AddFrame(frame_data.image, frame_data.stamp());
      apriltags.mutable_image()->CopyFrom(image_pb);
      event_bus_.AsyncSend(
          MakeEvent(frame_data.camera_model.frame_name() + "/apriltags",
                    apriltags, frame_data.stamp()));
    } break;
    case CameraPipelineCommand::RecordStart::MODE_APRILTAG_STABLE:
      LOG(INFO) << "Mode not support.";
      break;
    default:
      break;
  }

  if (!latest_command_.has_record_start()) {
    // Close may be called regardless of state.  If we were recording,
    // it closes the video file on the last chunk.
    video_file_writer_.Close();
    // Disposes of apriltag config (tag library, etc.)
    detector_.Close();
  }
}

CameraModel GridCameraModel() {
  auto model = Default1080HDCameraModel();
  model.set_image_height(model.image_height() * 1.5);
  return model;
}

MultiCameraPipeline::MultiCameraPipeline(EventBus& event_bus)
    : event_bus_(event_bus),
      work_(pool_.get_io_service()),
      udp_streamer_(event_bus, GridCameraModel(), VideoStreamer::MODE_MP4_UDP,
                    5000) {}

void MultiCameraPipeline::Start(size_t n_threads) { pool_.Start(n_threads); }

void MultiCameraPipeline::AddCamera(const CameraConfig& camera_config,
                                    const CameraModel& camera_model) {
  pipelines_.emplace(std::piecewise_construct,
                     std::forward_as_tuple(camera_model.frame_name()),
                     std::forward_as_tuple(event_bus_, pool_.get_io_service(),
                                           camera_config, camera_model));
}

void MultiCameraPipeline::Post(CameraPipelineCommand command) {
  for (auto& pipeline : pipelines_) {
    pipeline.second.Post(command);
  }
}

void MultiCameraPipeline::OnFrame(
    const std::vector<FrameData>& synced_frame_data) {
  CHECK(!synced_frame_data.empty());
  CHECK(!pool_.get_io_service().stopped());
  bool do_post = true;
  for (auto& pipeline : pipelines_) {
    if (pipeline.second.PostCount() > 0) {
      do_post = false;
    }
  }
  if (!do_post) {
    VLOG(1) << "pipeline full.";
  }

  std::vector<cv::Mat> images;
  for (const FrameData& frame : synced_frame_data) {
    if (do_post) {
      pipelines_.at(frame.camera_model.frame_name()).Post(frame);
    }
    images.push_back(frame.image);
  }
  auto grid_camera = GridCameraModel();
  udp_streamer_.AddFrame(
      ConstructGridImage(
          images,
          cv::Size(grid_camera.image_width(), grid_camera.image_height()), 2),
      synced_frame_data.front().stamp());
}

CameraPipelineClient::CameraPipelineClient(EventBus& bus)
    : io_service_(bus.get_io_service()),
      event_bus_(bus),

      multi_camera_pipeline_(event_bus_),
      multi_camera_(event_bus_) {
  event_bus_.GetEventSignal()->connect(
      std::bind(&CameraPipelineClient::on_event, this, std::placeholders::_1));

  event_bus_.AddSubscriptions(
      {// subscribe to logger commands for resource archive path changes,
       // should this just be default?
       std::string("^logger/.*"),
       // tracking camera commands, recording, etc.
       std::string("^camera_pipeline/command$")});

  CameraPipelineConfig config = ReadProtobufFromJsonFile<CameraPipelineConfig>(
      GetBucketAbsolutePath(Bucket::BUCKET_CONFIGURATIONS) / "camera.json");

  // This starts a thread per camera for processing.
  multi_camera_pipeline_.Start(config.camera_configs().size());

  for (const CameraConfig& camera_config : config.camera_configs()) {
    auto camera_model = multi_camera_.AddCameraConfig(camera_config);
    multi_camera_pipeline_.AddCamera(camera_config, camera_model);
  }

  multi_camera_.GetSynchronizedFrameDataSignal().connect(
      std::bind(&MultiCameraPipeline::OnFrame, &multi_camera_pipeline_,
                std::placeholders::_1));
}

void CameraPipelineClient::on_command(const CameraPipelineCommand& command) {
  latest_command_ = command;
  multi_camera_pipeline_.Post(latest_command_);
}

void CameraPipelineClient::on_event(const EventPb& event) {
  CameraPipelineCommand command;
  if (event.data().UnpackTo(&command)) {
    on_command(command);
    return;
  }
}

}  // namespace perception
}  // namespace farm_ng
