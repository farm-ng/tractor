#include <boost/asio/steady_timer.hpp>
#include <boost/signals2/signal.hpp>

#include "farm_ng/core/ipc.h"
#include "farm_ng/core/thread_pool.h"

#include "farm_ng/perception/apriltag.h"
#include "farm_ng/perception/frame_grabber.h"
#include "farm_ng/perception/time_series.h"
#include "farm_ng/perception/video_streamer.h"

#include "farm_ng/perception/camera_model.pb.h"
#include "farm_ng/perception/camera_pipeline.pb.h"
// #include "farm_ng/perception/geometry.pb.h"

typedef farm_ng::core::Event EventPb;
using farm_ng::core::Bucket;
using farm_ng::core::BUCKET_CONFIGURATIONS;
using farm_ng::core::EventBus;
using farm_ng::core::MakeEvent;
using farm_ng::core::ReadProtobufFromJsonFile;
using farm_ng::core::Subscription;
using farm_ng::core::ThreadPool;

namespace farm_ng {
namespace perception {

class MultiCameraSync {
  typedef boost::signals2::signal<void(
      const std::vector<FrameData>& synced_frames)>
      Signal;

 public:
  MultiCameraSync(EventBus& event_bus);
  CameraModel AddCameraConfig(const CameraConfig& camera_config);
  Signal& GetSynchronizedFrameDataSignal();

 private:
  std::vector<FrameData> CollectSynchronizedFrameData();
  void GetSynchronizedFrameData(const boost::system::error_code& error);
  void OnFrame(const FrameData& frame_data);

  EventBus& event_bus_;
  boost::asio::steady_timer timer_;
  std::vector<std::unique_ptr<FrameGrabber>> frame_grabbers_;
  std::mutex frame_series_mtx_;
  std::map<std::string, TimeSeries<FrameData>> frame_series_;
  google::protobuf::Timestamp latest_frame_stamp_;
  Signal signal_;
};

class SingleCameraPipeline {
 public:
  SingleCameraPipeline(EventBus& event_bus,
                       boost::asio::io_service& pool_service,
                       const CameraConfig& camera_config,
                       const CameraModel& camera_model);

  void Post(CameraPipelineCommand command);
  int PostCount() const;
  void Post(FrameData frame_data);
  void Compute(FrameData frame_data);

 private:
  EventBus& event_bus_;
  boost::asio::io_service::strand strand_;
  CameraModel camera_model_;
  ApriltagDetector detector_;
  VideoStreamer video_file_writer_;
  std::unique_ptr<VideoStreamer> udp_streamer_;
  CameraPipelineCommand latest_command_;
  std::atomic<int> post_count_;
};

CameraModel GridCameraModel();

class MultiCameraPipeline {
 public:
  MultiCameraPipeline(EventBus& event_bus);

  void Start(size_t n_threads);

  void AddCamera(const CameraConfig& camera_config,
                 const CameraModel& camera_model);

  void Post(CameraPipelineCommand command);

  void OnFrame(const std::vector<FrameData>& synced_frame_data);

 private:
  EventBus& event_bus_;
  ThreadPool pool_;
  boost::asio::io_service::work work_;
  VideoStreamer udp_streamer_;
  std::map<std::string, SingleCameraPipeline> pipelines_;
};

class TrackingCameraClient {
 public:
  TrackingCameraClient(EventBus& bus);
  void on_command(const CameraPipelineCommand& command);
  void on_event(const EventPb& event);

 private:
  boost::asio::io_service& io_service_;
  EventBus& event_bus_;
  MultiCameraPipeline multi_camera_pipeline_;
  MultiCameraSync multi_camera_;
  CameraPipelineConfig config_;
  CameraPipelineCommand latest_command_;
};

}  // namespace perception
}  // namespace farm_ng
