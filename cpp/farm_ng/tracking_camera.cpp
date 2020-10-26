#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/util/time_util.h>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <farm_ng/calibration/apriltag.h>
#include <farm_ng/calibration/base_to_camera_calibrator.h>
#include <farm_ng/calibration/visual_odometer.h>

#include <farm_ng/init.h>
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
using farm_ng_proto::tractor::v1::CameraModel;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::NamedSE3Pose;
using farm_ng_proto::tractor::v1::SteeringCommand;
using farm_ng_proto::tractor::v1::Subscription;
using farm_ng_proto::tractor::v1::TagConfig;
using farm_ng_proto::tractor::v1::TagLibrary;
using farm_ng_proto::tractor::v1::TrackingCameraCommand;
using farm_ng_proto::tractor::v1::TrackingCameraPoseFrame;
using farm_ng_proto::tractor::v1::Vec2;

namespace farm_ng {

struct ScopeGuard {
  ScopeGuard(std::function<void()> function) : function_(std::move(function)) {}
  ~ScopeGuard() { function_(); }
  std::function<void()> function_;
};

// Convert rs2::frame to cv::Mat
// https://raw.githubusercontent.com/IntelRealSense/librealsense/master/wrappers/opencv/cv-helpers.hpp
cv::Mat RS2FrameToMat(const rs2::frame& f) {
  using namespace cv;
  using namespace rs2;

  auto vf = f.as<video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();
  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    Mat r_bgr;
    cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
    return r_bgr;
  } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
    return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
    return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
  }

  throw std::runtime_error("Frame format is not supported yet!");
}

void SetVec3FromRs(farm_ng_proto::tractor::v1::Vec3* out, rs2_vector vec) {
  out->set_x(vec.x);
  out->set_y(vec.y);
  out->set_z(vec.z);
}

void SetQuatFromRs(farm_ng_proto::tractor::v1::Quaternion* out,
                   rs2_quaternion vec) {
  out->set_w(vec.w);
  out->set_x(vec.x);
  out->set_y(vec.y);
  out->set_z(vec.z);
}

void SetCameraModelFromRs(CameraModel* out, const rs2_intrinsics& intrinsics) {
  out->set_image_width(intrinsics.width);
  out->set_image_height(intrinsics.height);
  out->set_cx(intrinsics.ppx);
  out->set_cy(intrinsics.ppy);
  out->set_fx(intrinsics.fx);
  out->set_fy(intrinsics.fy);
  switch (intrinsics.model) {
    case RS2_DISTORTION_KANNALA_BRANDT4:
      out->set_distortion_model(CameraModel::DISTORTION_MODEL_KANNALA_BRANDT4);
      break;
    case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      out->set_distortion_model(
          CameraModel::DISTORTION_MODEL_INVERSE_BROWN_CONRADY);
      break;
    default:
      CHECK(false) << "Unhandled intrinsics model: "
                   << rs2_distortion_to_string(intrinsics.model);
  }
  for (int i = 0; i < 5; ++i) {
    out->add_distortion_coefficients(intrinsics.coeffs[i]);
  }
}

TrackingCameraPoseFrame::Confidence ToConfidence(int x) {
  switch (x) {
    case 0:
      return TrackingCameraPoseFrame::CONFIDENCE_FAILED;
    case 1:
      return TrackingCameraPoseFrame::CONFIDENCE_LOW;
    case 2:
      return TrackingCameraPoseFrame::CONFIDENCE_MEDIUM;
    case 3:
      return TrackingCameraPoseFrame::CONFIDENCE_HIGH;
    default:
      return TrackingCameraPoseFrame::CONFIDENCE_UNSPECIFIED;
  }
}

TrackingCameraPoseFrame ToPoseFrame(const rs2::pose_frame& rs_pose_frame) {
  TrackingCameraPoseFrame pose_frame;
  pose_frame.set_frame_number(rs_pose_frame.get_frame_number());
  *pose_frame.mutable_stamp_pose() =
      google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          rs_pose_frame.get_timestamp());
  auto pose_data = rs_pose_frame.get_pose_data();
  SetVec3FromRs(pose_frame.mutable_start_pose_current()->mutable_position(),
                pose_data.translation);
  SetQuatFromRs(pose_frame.mutable_start_pose_current()->mutable_rotation(),
                pose_data.rotation);
  SetVec3FromRs(pose_frame.mutable_velocity(), pose_data.velocity);
  SetVec3FromRs(pose_frame.mutable_acceleration(), pose_data.acceleration);
  SetVec3FromRs(pose_frame.mutable_angular_velocity(),
                pose_data.angular_velocity);
  SetVec3FromRs(pose_frame.mutable_angular_acceleration(),
                pose_data.angular_acceleration);
  pose_frame.set_tracker_confidence(ToConfidence(pose_data.tracker_confidence));
  pose_frame.set_mapper_confidence(ToConfidence(pose_data.mapper_confidence));
  return pose_frame;
}

EventPb ToNamedPoseEvent(const rs2::pose_frame& rs_pose_frame) {
  // TODO(ethanrublee) support front and rear cameras.
  NamedSE3Pose vodom_pose_t265;
  // here we distinguish where visual_odom frame by which camera it refers to,
  // will have to connect each camera pose to the mobile base with an extrinsic
  // transform
  vodom_pose_t265.set_frame_b("odometry/tracking_camera/front");
  vodom_pose_t265.set_frame_a("tracking_camera/front");
  auto pose_data = rs_pose_frame.get_pose_data();
  SetVec3FromRs(vodom_pose_t265.mutable_a_pose_b()->mutable_position(),
                pose_data.translation);
  SetQuatFromRs(vodom_pose_t265.mutable_a_pose_b()->mutable_rotation(),
                pose_data.rotation);

  Sophus::SE3d se3;
  ProtoToSophus(vodom_pose_t265.a_pose_b(), &se3);
  SophusToProto(se3.inverse(), vodom_pose_t265.mutable_a_pose_b());

  EventPb event =
      farm_ng::MakeEvent("pose/tracking_camera/front", vodom_pose_t265);
  *event.mutable_stamp() =
      google::protobuf::util::TimeUtil::MillisecondsToTimestamp(
          rs_pose_frame.get_timestamp());
  return event;
}

class TrackingCameraClient {
 public:
  TrackingCameraClient(EventBus& bus)
      : io_service_(bus.get_io_service()), event_bus_(bus) {
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

    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;
    // Add pose stream
    // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    auto profile = cfg.resolve(pipe_);
    auto tm2 = profile.get_device().as<rs2::tm2>();
    //    auto pose_sensor = tm2.first<rs2::pose_sensor>();

    // Start pipe and get camera calibrations
    const int fisheye_sensor_idx = 1;  // for the left fisheye lens of T265
    auto fisheye_stream =
        profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    auto fisheye_intrinsics =
        fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();

    SetCameraModelFromRs(&left_camera_model_, fisheye_intrinsics);
    left_camera_model_.set_frame_name(
        "tracking_camera/front/left");  // TODO Pass in constructor.
    detector_.reset(new ApriltagDetector(left_camera_model_, &event_bus_));
    LOG(INFO) << " intrinsics model: " << left_camera_model_.ShortDebugString();
    frame_video_writer_ = std::make_unique<VideoStreamer>(
        event_bus_, left_camera_model_, VideoStreamer::MODE_MP4_FILE);

    udp_streamer_ = std::make_unique<VideoStreamer>(
        event_bus_, left_camera_model_, VideoStreamer::MODE_MP4_UDP, 5000);
    // setting options for slam:
    // https://github.com/IntelRealSense/librealsense/issues/1011
    // and what to set:
    //  https://github.com/IntelRealSense/realsense-ros/issues/779 "
    // I would suggest leaving mapping enabled, but disabling
    // relocalization and jumping. This may avoid the conflict with
    // RTabMap while still giving good results."

    // pose_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 0);
    // pose_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 0);

    // Start pipeline with chosen configuration
    pipe_.start(cfg, std::bind(&TrackingCameraClient::frame_callback, this,
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
    frame_video_writer_->AddFrame(image, stamp);
  }

  void detect_apriltags(cv::Mat image, google::protobuf::Timestamp stamp) {
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
  }

  void RecordEveryApriltagFrame(cv::Mat image,
                                google::protobuf::Timestamp stamp) {
    auto apriltags = detector_->Detect(image, stamp);
    auto image_pb = frame_video_writer_->AddFrame(image, stamp);

    apriltags.mutable_image()->CopyFrom(image_pb);
    event_bus_.Send(farm_ng::MakeEvent(
        "calibrator/tracking_camera/front/apriltags", apriltags, stamp));
    event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/apriltags",
                                       apriltags, stamp));
  }

  // The callback is executed on a sensor thread and can be called
  // simultaneously from multiple sensors Therefore any modification to common
  // memory should be done under lock
  void frame_callback(const rs2::frame& frame) {
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
    } /* else if (rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>()) {
      // HACK disable for now, this harms the ipc perfomance.
      auto pose_data = pose_frame.get_pose_data();
      // Print the x, y, z values of the translation, relative to initial
      // position
      std::cout << "\r Device Position: " << std::setprecision(3) << std::fixed
                << pose_data.translation.x << " " << pose_data.translation.y
                << " " << pose_data.translation.z << " (meters)";
      event_bus_.Send(farm_ng::MakeEvent("tracking_camera/front/pose",
                                         ToPoseFrame(pose_frame)));
      event_bus_.Send(ToNamedPoseEvent(pose_frame));

  } */
  }
  boost::asio::io_service& io_service_;
  EventBus& event_bus_;
  std::unique_ptr<VideoStreamer> udp_streamer_;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe_;
  std::mutex mtx_realsense_state_;
  int count_ = 0;
  bool detection_in_progress_ = false;
  std::unique_ptr<ApriltagDetector> detector_;
  TrackingCameraCommand latest_command_;
  ApriltagsFilter tag_filter_;
  std::unique_ptr<VideoStreamer> frame_video_writer_;
  CameraModel left_camera_model_;

  std::optional<BaseToCameraModel> base_to_camera_model_;
  std::unique_ptr<VisualOdometer> vo_;
};
}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) {}

void Enumerate() {
  // Obtain a list of devices currently present on the system
  rs2::context ctx;

  auto devices_list = ctx.query_devices();
  size_t device_count = devices_list.size();
  if (!device_count) {
    LOG(INFO) << "No device detected. Is it plugged in?";
    return;
  }
  // Retrieve the viable devices
  std::vector<rs2::device> devices;

  for (auto i = 0u; i < device_count; i++) {
    try {
      auto dev = devices_list[i];
      devices.emplace_back(dev);
    } catch (const std::exception& e) {
      std::cout << "Could not create device - " << e.what()
                << " . Check SDK logs for details" << std::endl;
    } catch (...) {
      std::cout << "Failed to created device. Check SDK logs for details"
                << std::endl;
    }
  }
  for (auto i = 0u; i < devices.size(); ++i) {
    auto dev = devices[i];

    std::cout << std::left << std::setw(30)
              << dev.get_info(RS2_CAMERA_INFO_NAME) << std::setw(20)
              << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::setw(20)
              << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
  }
}
int Main(farm_ng::EventBus& bus) {
  Enumerate();
  return 0;
  farm_ng::TrackingCameraClient client(bus);
  try {
    bus.get_io_service().run();
  } catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
