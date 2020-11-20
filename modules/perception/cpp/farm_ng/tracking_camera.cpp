#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/util/time_util.h>
#include <boost/asio/steady_timer.hpp>

#include <opencv2/opencv.hpp>

#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/apriltag.pb.h"
#include "farm_ng/perception/camera_model.h"
#include "farm_ng/perception/frame_grabber.h"
#include "farm_ng/perception/image_utils.h"
#include "farm_ng/perception/sophus_protobuf.h"
#include "farm_ng/perception/time_series.h"
#include "farm_ng/perception/video_streamer.h"

#include "farm_ng/perception/camera_pipeline.pb.h"
#include "farm_ng/perception/geometry.pb.h"

typedef farm_ng::core::Event EventPb;
using farm_ng::core::Bucket;
using farm_ng::core::BUCKET_CONFIGURATIONS;
using farm_ng::core::EventBus;
using farm_ng::core::MakeEvent;
using farm_ng::core::ReadProtobufFromJsonFile;
using farm_ng::core::Subscription;

namespace farm_ng {
namespace perception {

class MultiCameraSync {
 public:
  typedef boost::signals2::signal<void(
      const std::vector<FrameData>& synced_frames)>
      Signal;

  MultiCameraSync(EventBus& event_bus)
      : event_bus_(event_bus), timer_(event_bus.get_io_service()) {}

  CameraModel AddCameraConfig(const CameraConfig& camera_config) {
    frame_grabbers_.emplace_back(
        FrameGrabber::MakeFrameGrabber(event_bus_, camera_config));
    frame_grabbers_.back()->VisualFrameSignal().connect(
        std::bind(&MultiCameraSync::OnFrame, this, std::placeholders::_1));
    return frame_grabbers_.back()->GetCameraModel();
  }
  Signal& GetSynchronizedFrameDataSignal() { return signal_; }

 private:
  std::vector<FrameData> CollectSynchronizedFrameData() {
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
  void GetSynchronizedFrameData(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "Synchronized frame timer error (frame sync may not be "
                      "keeping up): "
                   << error;
      return;
    }
    signal_(CollectSynchronizedFrameData());
  }

  void OnFrame(const FrameData& frame_data) {
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

 private:
  EventBus& event_bus_;

  boost::asio::steady_timer timer_;

  std::vector<std::unique_ptr<FrameGrabber>> frame_grabbers_;
  std::mutex frame_series_mtx_;
  std::map<std::string, TimeSeries<FrameData>> frame_series_;
  google::protobuf::Timestamp latest_frame_stamp_;
  Signal signal_;
};

class ThreadPool {
 public:
  ThreadPool() {}

  void Stop() { io_service_.stop(); }

  void Start(size_t n_threads) {
    CHECK_GT(n_threads, 0);
    CHECK(threads_.empty()) << "ThreadPool already started. Call Join().";
    io_service_.reset();
    for (size_t i = 0; i < n_threads; ++i) {
      threads_.emplace_back([this]() { io_service_.run(); });
    }
  }

  void Join() {
    for (auto& thread : threads_) {
      thread.join();
    }
    threads_.clear();
  }

  boost::asio::io_service& get_io_service() { return io_service_; }

 private:
  boost::asio::io_service io_service_;
  std::vector<std::thread> threads_;
};

class SingleCameraPipeline {
 public:
  SingleCameraPipeline(EventBus& event_bus,
                       boost::asio::io_service& pool_service,
                       const CameraConfig& camera_config,
                       const CameraModel& camera_model)
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

  void Post(CameraPipelineCommand command) {
    strand_.post([this, command] {
      latest_command_.CopyFrom(command);
      LOG(INFO) << "Camera pipeline: " << camera_model_.frame_name()
                << " command: " << latest_command_.ShortDebugString();
    });
  }

  int PostCount() const { return post_count_; }
  void Post(FrameData frame_data) {
    post_count_++;
    strand_.post([this, frame_data] {
      Compute(frame_data);
      post_count_--;
      CHECK_GE(post_count_, 0);
    });
  }

  void Compute(FrameData frame_data) {
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

  EventBus& event_bus_;
  boost::asio::strand strand_;
  CameraModel camera_model_;
  ApriltagDetector detector_;
  VideoStreamer video_file_writer_;
  std::unique_ptr<VideoStreamer> udp_streamer_;
  CameraPipelineCommand latest_command_;
  std::atomic<int> post_count_;
};
CameraModel GridCameraModel() {
  auto model = Default1080HDCameraModel();
  model.set_image_height(model.image_height() * 1.5);
  return model;
}
class MultiCameraPipeline {
 public:
  MultiCameraPipeline(EventBus& event_bus)
      : event_bus_(event_bus),
        work_(pool_.get_io_service()),
        udp_streamer_(event_bus, GridCameraModel(), VideoStreamer::MODE_MP4_UDP,
                      5000) {}

  void Start(size_t n_threads) { pool_.Start(n_threads); }

  void AddCamera(const CameraConfig& camera_config,
                 const CameraModel& camera_model) {
    pipelines_.emplace(std::piecewise_construct,
                       std::forward_as_tuple(camera_model.frame_name()),
                       std::forward_as_tuple(event_bus_, pool_.get_io_service(),
                                             camera_config, camera_model));
  }

  void Post(CameraPipelineCommand command) {
    for (auto& pipeline : pipelines_) {
      pipeline.second.Post(command);
    }
  }

  void OnFrame(const std::vector<FrameData>& synced_frame_data) {
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
  EventBus& event_bus_;

  ThreadPool pool_;
  boost::asio::io_service::work work_;
  VideoStreamer udp_streamer_;
  std::map<std::string, SingleCameraPipeline> pipelines_;
};

class TrackingCameraClient {
 public:
  TrackingCameraClient(EventBus& bus)
      : io_service_(bus.get_io_service()),
        event_bus_(bus),

        multi_camera_pipeline_(event_bus_),
        multi_camera_(event_bus_) {
    event_bus_.GetEventSignal()->connect(std::bind(
        &TrackingCameraClient::on_event, this, std::placeholders::_1));

    event_bus_.AddSubscriptions(
        {// subscribe to logger commands for resource archive path changes,
         // should this just be default?
         std::string("^logger/.*"),
         // subscribe to steering commands to toggle new goals for VO
         std::string("^steering$"),
         // tracking camera commands, recording, etc.
         std::string("^tracking_camera/command$"),
         // tractor states for VO
         std::string("^tractor_state$")});

    CameraPipelineConfig config =
        ReadProtobufFromJsonFile<CameraPipelineConfig>(
            GetBucketAbsolutePath(Bucket::BUCKET_CONFIGURATIONS) /
            "camera.json");

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

  void on_command(const CameraPipelineCommand& command) {
    latest_command_ = command;
    multi_camera_pipeline_.Post(latest_command_);
  }

  void on_event(const EventPb& event) {
    CameraPipelineCommand command;
    if (event.data().UnpackTo(&command)) {
      on_command(command);
      return;
    }
  }

  boost::asio::io_service& io_service_;
  EventBus& event_bus_;
  MultiCameraPipeline multi_camera_pipeline_;
  MultiCameraSync multi_camera_;
  CameraPipelineConfig config_;
  CameraPipelineCommand latest_command_;
};

}  // namespace perception
}  // namespace farm_ng

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  try {
    farm_ng::perception::TrackingCameraClient client(bus);
    bus.get_io_service().run();
  } catch (...) {
  }
  return 0;
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
