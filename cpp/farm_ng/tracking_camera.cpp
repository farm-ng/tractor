#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/util/time_util.h>
#include <boost/asio/steady_timer.hpp>

#include <opencv2/opencv.hpp>

#include <farm_ng/calibration/apriltag.h>
#include <farm_ng/calibration/camera_model.h>

#include <farm_ng/calibration/base_to_camera_calibrator.h>
#include <farm_ng/calibration/visual_odometer.h>

#include <farm_ng/init.h>
#include <farm_ng/intel_frame_grabber.h>
#include <farm_ng/ipc.h>
#include <farm_ng/video_streamer.h>

#include <farm_ng/sophus_protobuf.h>

#include <farm_ng_proto/tractor/v1/apriltag.pb.h>
#include <farm_ng_proto/tractor/v1/geometry.pb.h>
#include <farm_ng_proto/tractor/v1/steering.pb.h>
#include <farm_ng_proto/tractor/v1/tracking_camera.pb.h>
#include <farm_ng_proto/tractor/v1/tractor.pb.h>

#include <farm_ng_proto/tractor/v1/calibrate_base_to_camera.pb.h>

typedef farm_ng_proto::tractor::v1::Event EventPb;

using farm_ng_proto::tractor::v1::ApriltagConfig;
using farm_ng_proto::tractor::v1::ApriltagDetection;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::BUCKET_CONFIGURATIONS;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraResult;
using farm_ng_proto::tractor::v1::CameraConfig;
using farm_ng_proto::tractor::v1::CameraModel;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::SteeringCommand;
using farm_ng_proto::tractor::v1::Subscription;
using farm_ng_proto::tractor::v1::TagConfig;
using farm_ng_proto::tractor::v1::TagLibrary;
using farm_ng_proto::tractor::v1::TrackingCameraCommand;
using farm_ng_proto::tractor::v1::TrackingCameraConfig;
using farm_ng_proto::tractor::v1::TrackingCameraPoseFrame;
using farm_ng_proto::tractor::v1::Vec2;

namespace farm_ng {

struct ScopeGuard {
  ScopeGuard(std::function<void()> function) : function_(std::move(function)) {}
  ~ScopeGuard() { function_(); }
  std::function<void()> function_;
};

class MultiCameraSync {
 public:
  typedef boost::signals2::signal<void(
      const std::vector<FrameData>& synced_frames)>
      Signal;

  MultiCameraSync(EventBus& event_bus)
      : event_bus_(event_bus), timer_(event_bus.get_io_service()) {}

  void AddCameraConfig(const CameraConfig& camera_config) {
    frame_grabbers_.emplace_back(
        std::make_unique<IntelFrameGrabber>(event_bus_, camera_config));
    frame_grabbers_.back()->VisualFrameSignal().connect(
        std::bind(&MultiCameraSync::OnFrame, this, std::placeholders::_1));
  }
  Signal& GetSynchronizedFrameDataSignal() { return signal_; }

 private:
  std::vector<FrameData> CollectSynchronizedFrameData() {
    std::vector<FrameData> synced_frames;
    std::lock_guard<std::mutex> lock(frame_series_mtx_);

    auto begin_range =
        latest_frame_stamp_ -
        google::protobuf::util::TimeUtil::MillisecondsToDuration(1000.0 / 7);

    auto end_range =
        latest_frame_stamp_ +
        google::protobuf::util::TimeUtil::MillisecondsToDuration(1000.0 / 7);

    for (const auto& grabber : frame_grabbers_) {
      const auto& frame_name = grabber->GetCameraModel().frame_name();
      const auto& series = frame_series_[frame_name];
      auto range = series.find_range(begin_range, end_range);

      auto closest = range.first;
      double score = 1e10;
      while (range.first != range.second) {
        double score_i =
            google::protobuf::util::TimeUtil::DurationToMilliseconds(
                range.first->stamp() - latest_frame_stamp_);
        if (score_i < score) {
          score = score_i;
          closest = range.first;
        }
        range.first++;
      }
      if (closest != series.end()) {
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

  std::vector<std::unique_ptr<IntelFrameGrabber>> frame_grabbers_;
  std::mutex frame_series_mtx_;
  std::map<std::string, TimeSeries<FrameData>> frame_series_;
  google::protobuf::Timestamp latest_frame_stamp_;
  Signal signal_;
};

class MultiCameraPipeline {
 public:
  MultiCameraPipeline(EventBus& event_bus)
      : event_bus_(event_bus),
        udp_streamer_(event_bus, Default1080HDCameraModel(),
                      VideoStreamer::MODE_MP4_UDP, 5000) {}

  void OnFrame(const std::vector<FrameData>& synced_frame_data) {
    cv::Mat grid_image = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC3);
    int n_cols = 2;
    int n_rows = std::ceil(synced_frame_data.size() / float(n_cols));
    size_t i = 0;
    int target_width = grid_image.size().width / n_cols;
    int target_height = grid_image.size().height / n_rows;
    for (int row = 0; row < n_rows && i < synced_frame_data.size(); ++row) {
      for (int col = 0; col < n_cols && i < synced_frame_data.size();
           ++col, ++i) {
        cv::Mat image_i = synced_frame_data[i].image;
        float width_ratio = target_width / float(image_i.size().width);
        float height_ratio = target_height / float(image_i.size().height);
        float resize_ratio = std::min(width_ratio, height_ratio);
        int i_width = image_i.size().width * resize_ratio;
        int i_height = image_i.size().height * resize_ratio;
        cv::Rect roi(col * target_width, row * target_height, i_width,
                     i_height);
        cv::Mat color_i;
        if (image_i.channels() == 1) {
          cv::cvtColor(image_i, color_i, cv::COLOR_GRAY2BGR);
        } else {
          color_i = image_i;
          CHECK_EQ(color_i.channels(), 3);
        }

        cv::resize(color_i, grid_image(roi), roi.size());
      }
    }
    udp_streamer_.AddFrame(grid_image, synced_frame_data.front().stamp());
  }
  EventBus& event_bus_;
  VideoStreamer udp_streamer_;
};

// class CameraPipeline {
//  public:
//   CameraPipline(EventBus& event_bus, const CameraConfig& camera_config,
//                 const CameraModel& camera_model)
//       : event_bus_(event_bus),
//       camera_config_(camera_config),
//         camera_model_(camera_model),
//         detector_(camera_model, &event_bus),
//          {}

//   void OnFrame(const CameraConfig& camera_config, const CameraModel& model,
//                const cv::Mat& image, const google::protobuf::Timestamp&
//                stamp) {

//   }
//   EventBus& event_bus_;
//   CameraConfig camera_config_;
//   CameraModel camera_model_;
//   ApriltagDetector detector_;
//   VideoStreamer video_file_writer_;
//   std::optional<VideoStreamer> udp_streamer_
// };
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

    auto base_to_camera_path =
        GetBucketRelativePath(Bucket::BUCKET_BASE_TO_CAMERA_MODELS) /
        "base_to_camera.json";

    if (boost::filesystem::exists(base_to_camera_path)) {
      auto base_to_camera_result =
          ReadProtobufFromJsonFile<CalibrateBaseToCameraResult>(
              base_to_camera_path);

      base_to_camera_model_ = ReadProtobufFromResource<BaseToCameraModel>(
          base_to_camera_result.base_to_camera_model_solved());
      base_to_camera_model_->clear_samples();
      LOG(INFO) << "Loaded base_to_camera_model_"
                << base_to_camera_model_->ShortDebugString();
    }

    TrackingCameraConfig config =
        ReadProtobufFromJsonFile<TrackingCameraConfig>(
            GetBucketAbsolutePath(Bucket::BUCKET_CONFIGURATIONS) /
            "camera.json");

    for (const CameraConfig& camera_config : config.camera_configs()) {
      multi_camera_.AddCameraConfig(camera_config);
    }

    multi_camera_.GetSynchronizedFrameDataSignal().connect(
        std::bind(&MultiCameraPipeline::OnFrame, multi_camera_pipeline_,
                  std::placeholders::_1));
  }

  void on_command(const TrackingCameraCommand& command) {
    latest_command_ = command;
    LOG(INFO) << "Got command: " << latest_command_.ShortDebugString();
  }

  void on_event(const EventPb& event) {
    TrackingCameraCommand command;
    if (event.data().UnpackTo(&command)) {
      on_command(command);
      return;
    }
    TractorState state;
    if (event.data().UnpackTo(&state) && vo_) {
      BaseToCameraModel::WheelMeasurement wheel_measurement;
      CopyTractorStateToWheelState(state, &wheel_measurement);
      // LOG(INFO) <<wheel_measurement.ShortDebugString();
      vo_->AddWheelMeasurements(wheel_measurement);
      return;
    }
    SteeringCommand steering;
    if (vo_ && event.data().UnpackTo(&steering)) {
      if (steering.reset_goal()) {
        vo_->SetGoal();
      }
      if (std::abs(steering.angular_velocity()) > 0.0) {
        vo_->AdjustGoalAngle(steering.angular_velocity() * 1 / 50.0);
      }
    }
  }

  void record_every_frame(cv::Mat image, google::protobuf::Timestamp stamp) {
    // frame_video_writer_->AddFrame(image, stamp);
  }

  void detect_apriltags(cv::Mat image, google::protobuf::Timestamp stamp) {
#if 0
    auto image_pb = frame_video_writer_->AddFrame(image, stamp);

    auto apriltags = detector_->Detect(image, stamp);

    if (tag_filter_.AddApriltags(apriltags)) {
      auto resource_path = GetUniqueArchiveResource(
          "tracking_camera_left_apriltag", "png", "image/png");
      apriltags.mutable_image()->mutable_resource()->CopyFrom(
          resource_path.first);

      LOG(INFO) << "Writing to : " << resource_path.second;
      CHECK(cv::imwrite(resource_path.second.string(), image))
          << "Failed to write image to: " << resource_path.second;

      event_bus_.Send(farm_ng::MakeEvent(
          "calibrator/tracking_camera/front/apriltags", apriltags, stamp));
    }

    event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/apriltags",
                                       apriltags, stamp));
#endif
  }

  void RecordEveryApriltagFrame(cv::Mat image,
                                google::protobuf::Timestamp stamp) {
#if 0
    auto apriltags = detector_->Detect(image, stamp);
    auto image_pb = frame_video_writer_->AddFrame(image, stamp);

    apriltags.mutable_image()->CopyFrom(image_pb);
    event_bus_.Send(farm_ng::MakeEvent(
        "calibrator/tracking_camera/front/apriltags", apriltags, stamp));
    event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/apriltags",
                                       apriltags, stamp));
#endif
  }
#if 0
  // The callback is executed on a sensor thread and can be called
  // simultaneously from multiple sensors Therefore any modification to common
  // memory should be done under lock
  void FrameCallback(const CameraConfig& camera_config,
                     const CameraModel& model, const cv::Mat& image,
                     const google::protobuf::Timestamp& stamp) {
    if (rs2::frameset fs = frame.as<rs2::frameset>()) {
      rs2::video_frame fisheye_frame = fs.get_fisheye_frame(0);
      // Add a reference to fisheye_frame, cause we're scheduling
      // april tag detection for later.
      fisheye_frame.keep();
      cv::Mat frame_0 = RS2FrameToMat(fisheye_frame);

      // lock for rest of scope, so we can edit some member state.
      std::lock_guard<std::mutex> lock(mtx_realsense_state_);

      // we only want to schedule detection if we're not currently
      // detecting.  Apriltag detection takes >30ms on the nano.
      if (!detection_in_progress_) {
        detection_in_progress_ = true;

        auto stamp = google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
            fisheye_frame.get_timestamp());

        // schedule april tag detection, do it as frequently as possible.
        io_service_.post([this, fisheye_frame, stamp] {
          // note this function is called later, in main thread, via
          // io_service_.run();

          ScopeGuard guard([this] {
            // signal that we're done detecting, so can post another frame for
            // detection.
            std::lock_guard<std::mutex> lock(mtx_realsense_state_);
            detection_in_progress_ = false;
          });
          if (!vo_ && base_to_camera_model_) {
            vo_.reset(new VisualOdometer(left_camera_model_,
                                         *base_to_camera_model_, 100));
            LOG(INFO) << "Starting VO.";
          }
          cv::Mat frame_0 = RS2FrameToMat(fisheye_frame);
          cv::Mat send_frame;

          if (vo_) {
            VisualOdometerResult result = vo_->AddImage(frame_0, stamp);
            event_bus_.Send(MakeEvent("pose/tractor/base/goal",
                                      result.base_pose_goal, stamp));
            send_frame = vo_->GetDebugImage().clone();
          }

          if (send_frame.empty()) {
            cv::cvtColor(frame_0, send_frame, cv::COLOR_GRAY2BGR);
          }
          // TODO(ethanrublee) base this on view direction?
          // cv::flip(send_frame, send_frame, -1);
          count_ = (count_ + 1) % 100;

          if (count_ % 3 == 0) {
            udp_streamer_->AddFrame(send_frame, stamp);
          }
          if (!latest_command_.has_record_start()) {
            // Close may be called regardless of state.  If we were recording,
            // it closes the video file on the last chunk.
            frame_video_writer_->Close();
            // Disposes of apriltag config (tag library, etc.)
            detector_->Close();
            return;
          }

          switch (latest_command_.record_start().mode()) {
            case TrackingCameraCommand::RecordStart::MODE_EVERY_FRAME:
              record_every_frame(frame_0, stamp);
              break;
            case TrackingCameraCommand::RecordStart::MODE_APRILTAG_STABLE:
              detect_apriltags(frame_0, stamp);
              break;

            case TrackingCameraCommand::RecordStart::MODE_EVERY_APRILTAG_FRAME:
              RecordEveryApriltagFrame(frame_0, stamp);
              break;
            default:
              break;
          }
        });
      }
    }
  }
#endif
  boost::asio::io_service& io_service_;
  EventBus& event_bus_;
  MultiCameraPipeline multi_camera_pipeline_;
  MultiCameraSync multi_camera_;

  // rs2::context ctx_;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  // rs2::pipeline pipe_;
  std::mutex mtx_realsense_state_;
  int count_ = 0;
  bool detection_in_progress_ = false;
  std::unique_ptr<ApriltagDetector> detector_;
  TrackingCameraCommand latest_command_;
  ApriltagsFilter tag_filter_;

  CameraModel left_camera_model_;
  TrackingCameraConfig config_;
  std::map<std::string, CameraModel> camera_models_;
  std::optional<BaseToCameraModel> base_to_camera_model_;
  std::unique_ptr<VisualOdometer> vo_;
};  // namespace farm_ng
}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) {}

int Main(farm_ng::EventBus& bus) {
  try {
    farm_ng::TrackingCameraClient client(bus);
    bus.get_io_service().run();
  } catch (...) {
  }
  return 0;
}

int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
