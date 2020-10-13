// ~/code/tractor/build/cpp/farm_ng/tracking_study --apriltag_rig_result
// ~/code/tractor-data/apriltag_rig_models/field_tags_rig_1010.json
// --base_to_camera_result
// ~/code/tractor-data/base_to_camera_models/base_to_camera.json
// --video_dataset_result
// ~/code/tractor-data/video_datasets/tractor0003_20201011_field_tags_01_video02.json
#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng/event_log_reader.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "farm_ng/calibration/base_to_camera_calibrator.h"
#include "farm_ng/calibration/camera_model.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/time_series.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/calibrate_apriltag_rig.pb.h"
#include "farm_ng_proto/tractor/v1/calibrate_base_to_camera.pb.h"
#include "farm_ng_proto/tractor/v1/calibrator.pb.h"
#include "farm_ng_proto/tractor/v1/capture_calibration_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"
#include "farm_ng_proto/tractor/v1/tractor.pb.h"

typedef farm_ng_proto::tractor::v1::Event EventPb;
using farm_ng_proto::tractor::v1::ApriltagDetections;
using farm_ng_proto::tractor::v1::BaseToCameraModel;
using farm_ng_proto::tractor::v1::CalibrateApriltagRigResult;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraConfiguration;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraResult;
using farm_ng_proto::tractor::v1::CalibrateBaseToCameraStatus;
using farm_ng_proto::tractor::v1::TractorState;

using farm_ng_proto::tractor::v1::CaptureCalibrationDatasetResult;
using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::Image;
using farm_ng_proto::tractor::v1::Resource;

using farm_ng_proto::tractor::v1::MonocularApriltagRigModel;
using farm_ng_proto::tractor::v1::SolverStatus;
using farm_ng_proto::tractor::v1::SolverStatus_Name;
using farm_ng_proto::tractor::v1::TractorState;
using farm_ng_proto::tractor::v1::ViewDirection;
using farm_ng_proto::tractor::v1::ViewDirection_Name;
using farm_ng_proto::tractor::v1::ViewDirection_Parse;

DEFINE_string(video_dataset_result, "",
              "The path to a serialized CaptureVideoDatasetResult");
DEFINE_string(apriltag_rig_result, "",
              "The path to a serialized ApriltagRigCalibrationResult");
DEFINE_string(base_to_camera_result, "",
              "The path to a serialized CalibrateBaseToCameraResult");

DEFINE_bool(zero_indexed, true, "Data recorded with zero indexed video frame numbers?  This is a hack that should be removed once data format is stable.");
namespace farm_ng {

class ImageLoader {
 public:
  void OpenVideo(const Resource& resource) {
    CHECK_EQ(resource.content_type(), "video/mp4");

    if (!capture_) {
      video_name_ = (GetBlobstoreRoot() / resource.path()).string();
      capture_.reset(new cv::VideoCapture(video_name_));
    } else {
      if (video_name_ != (GetBlobstoreRoot() / resource.path()).string()) {
        capture_.reset(nullptr);
        OpenVideo(resource);
      }
    }
    CHECK(capture_->isOpened()) << "Video is not opened: " << video_name_;
  }

  cv::Mat LoadImage(const Image& image) {
    cv::Mat frame;
    if (image.resource().content_type() == "video/mp4") {
      OpenVideo(image.resource());
      LOG(INFO) << image.resource().path()
                << " frame number: " << image.frame_number().value();

      int frame_number =  image.frame_number().value();
      if(!FLAGS_zero_indexed) {
	CHECK_GT(frame_number, 0);
	frame_number -= 1;
      }
      capture_->set(cv::CAP_PROP_POS_FRAMES, frame_number);
      CHECK_EQ(frame_number,
               uint32_t(capture_->get(cv::CAP_PROP_POS_FRAMES)));
      *capture_ >> frame;
    } else {
      frame =
          cv::imread((GetBlobstoreRoot() / image.resource().path()).string(),
                     cv::IMREAD_UNCHANGED);
    }
    CHECK(!frame.empty());
    CHECK_EQ(frame.size().width, image.camera_model().image_width());
    CHECK_EQ(frame.size().height, image.camera_model().image_height());
    return frame;
  }
  std::unique_ptr<cv::VideoCapture> capture_;
  std::string video_name_;
};

class TrackingStudyProgram {
 public:
  TrackingStudyProgram(EventBus& bus)
      : bus_(bus), timer_(bus_.get_io_service()) {
    on_timer(boost::system::error_code());
  }

  int run() {
    auto dataset_result = ReadProtobufFromJsonFile<CaptureVideoDatasetResult>(
        FLAGS_video_dataset_result);

    auto rig_result = ReadProtobufFromJsonFile<CalibrateApriltagRigResult>(
        FLAGS_apriltag_rig_result);

    auto base_to_camera_result =
        ReadProtobufFromJsonFile<CalibrateBaseToCameraResult>(
            FLAGS_base_to_camera_result);

    auto base_to_camera_model = ReadProtobufFromResource<BaseToCameraModel>(
        base_to_camera_result.base_to_camera_model_solved());

    auto rig_model = ReadProtobufFromResource<MonocularApriltagRigModel>(
        rig_result.monocular_apriltag_rig_solved());

    EventLogReader log_reader(dataset_result.dataset());

    TimeSeries<BaseToCameraModel::WheelMeasurement> wheel_measurements;
    TimeSeries<farm_ng_proto::tractor::v1::Event> images;

    double wheel_baseline = base_to_camera_model.wheel_baseline();
    double wheel_radius = base_to_camera_model.wheel_radius();
    LOG(INFO) << "wheel baseline=" << wheel_baseline
              << " wheel_radius=" << wheel_radius;
    auto base_pose_camera_pb = base_to_camera_model.base_pose_camera();
    LOG(INFO) << base_pose_camera_pb.ShortDebugString();
    Sophus::SE3d base_pose_camera;
    ProtoToSophus(base_pose_camera_pb.a_pose_b(), &base_pose_camera);

    while (true) {
      try {
        auto event = log_reader.ReadNext();
        TractorState state;
        if (event.data().UnpackTo(&state)) {
          BaseToCameraModel::WheelMeasurement wheel_measurement;
          CopyTractorStateToWheelState(state, &wheel_measurement);
          wheel_measurements.insert(wheel_measurement);
        }
        if (event.data().Is<Image>()) {
          images.insert(event);
        }
        bus_.get_io_service().poll();

      } catch (std::runtime_error& e) {
        LOG(INFO) << e.what();
        break;
      }
    }

    ImageLoader image_loader;
    std::optional<google::protobuf::Timestamp> last_image_stamp;

    auto begin_event = images.begin();
    auto end_event = images.end();

    std::string encoder_x264(" x264enc ! ");

    std::string video_writer_dev =
        std::string("appsrc !") + " videoconvert ! " + encoder_x264 +
        " mp4mux ! filesink location=" + "/tmp/out.mp4";
    LOG(INFO) << "writing video with: " << video_writer_dev;

    std::unique_ptr<cv::VideoWriter> writer;
    for (auto& event : images) {
      Image image;

      if (event.data().UnpackTo(&image)) {
        if (!writer) {
          writer.reset(
              new cv::VideoWriter(video_writer_dev,
                                  0,   // fourcc
                                  30,  // fps
                                  cv::Size(image.camera_model().image_width(),
                                           image.camera_model().image_height()),
                                  true));
        }

        cv::Mat frame = image_loader.LoadImage(image);
        if (frame.channels() == 1) {
          cv::cvtColor(frame.clone(), frame, cv::COLOR_GRAY2RGB);
        }
        auto last_image_stamp = event.stamp();
        Eigen::Vector2d last_point_image = ProjectPointToPixel(
            image.camera_model(),
            base_pose_camera.inverse() * Eigen::Vector3d(0, 0, 0));
        Sophus::SE3d odometry_pose_base = Sophus::SE3d::rotX(0.0);
        for (auto it = begin_event;
             it != end_event && std::distance(begin_event, it) < 1000; ++it) {
          auto next_stamp = it->stamp();
          auto states =
              wheel_measurements.find_range(last_image_stamp, next_stamp);

          auto base_pose_basep = TractorStartPoseTractorEnd(
              wheel_radius, wheel_baseline, states.first, states.second);
          odometry_pose_base = odometry_pose_base * base_pose_basep;
          Eigen::Vector3d point_camera =
              base_pose_camera.inverse() *
              (odometry_pose_base * Eigen::Vector3d(0.0, 0.0, 0.0));

          Eigen::Vector2d point_image =
              ProjectPointToPixel(image.camera_model(), point_camera);
          cv::circle(frame, cv::Point(point_image.x(), point_image.y()), 2,
                     cv::Scalar(0, 0, 255));

          cv::line(frame, cv::Point(last_point_image.x(), last_point_image.y()),
                   cv::Point(point_image.x(), point_image.y()),
                   cv::Scalar(255, 255, 0));

          last_point_image = point_image;
          last_image_stamp = next_stamp;
        }

        last_image_stamp = event.stamp();
        cv::flip(frame, frame, -1);
        cv::imshow(event.name(), frame);
        writer->write(frame);
        cv::waitKey(10);
        bus_.get_io_service().poll();
        begin_event++;
      }
    }

    return 0;
  }

  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(std::bind(&TrackingStudyProgram::on_timer, this,
                                std::placeholders::_1));
  }

 private:
  EventBus& bus_;
  boost::asio::deadline_timer timer_;
};  // namespace farm_ng

}  // namespace farm_ng

void Cleanup(farm_ng::EventBus& bus) { LOG(INFO) << "Cleanup."; }

int Main(farm_ng::EventBus& bus) {
  farm_ng::TrackingStudyProgram program(bus);
  return program.run();
}
int main(int argc, char* argv[]) {
  return farm_ng::Main(argc, argv, &Main, &Cleanup);
}
