/*
tractor/build/cpp/farm_ng/tracking_study \
 --base_to_camera_result \
 tractor-data/base_to_camera_models/base_to_camera.json \
 --video_dataset_result \
 tractor-data/video_datasets/tractor0003_20201011_field_loop_01_notags.json \
  --zero_indexed=False
 */
#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "farm_ng/blobstore.h"
#include "farm_ng/event_log_reader.h"
#include "farm_ng/image_loader.h"
#include "farm_ng/init.h"
#include "farm_ng/ipc.h"

#include "farm_ng/calibration/base_to_camera_calibrator.h"
#include "farm_ng/calibration/visual_odometer.h"

#include "farm_ng_proto/tractor/v1/calibrate_base_to_camera.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"

using farm_ng_proto::tractor::v1::CalibrateBaseToCameraResult;
using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::TractorState;

DEFINE_string(video_dataset_result, "",
              "The path to a serialized CaptureVideoDatasetResult");
DEFINE_string(base_to_camera_result, "",
              "The path to a serialized CalibrateBaseToCameraResult");

DEFINE_bool(zero_indexed, true,
            "Data recorded with zero indexed video frame numbers?  This is a "
            "hack that should be removed once data format is stable.");
namespace farm_ng {

class TrackingStudyProgram {
 public:
  TrackingStudyProgram(EventBus& bus)
      : bus_(bus), timer_(bus_.get_io_service()) {
    on_timer(boost::system::error_code());
  }

  int run() {
    auto dataset_result = ReadProtobufFromJsonFile<CaptureVideoDatasetResult>(
        FLAGS_video_dataset_result);

    auto base_to_camera_result =
        ReadProtobufFromJsonFile<CalibrateBaseToCameraResult>(
            FLAGS_base_to_camera_result);

    auto base_to_camera_model = ReadProtobufFromResource<BaseToCameraModel>(
        base_to_camera_result.base_to_camera_model_solved());

    EventLogReader log_reader(dataset_result.dataset());

    std::unique_ptr<VisualOdometer> vo;
    ImageLoader image_loader(FLAGS_zero_indexed);

    int skip_frame_counter = 0;
    while (true) {
      try {
        auto event = log_reader.ReadNext();
        TractorState state;
        if (event.data().UnpackTo(&state) && vo) {
          BaseToCameraModel::WheelMeasurement wheel_measurement;
          CopyTractorStateToWheelState(state, &wheel_measurement);
          vo->AddWheelMeasurements(wheel_measurement);
        }
        Image image;
        if (event.data().UnpackTo(&image) && skip_frame_counter++ % 1 == 0) {
          if (!vo) {
            vo.reset(new VisualOdometer(image.camera_model(),
                                        base_to_camera_model, 100));
          }
          cv::Mat gray = image_loader.LoadImage(image);

          cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

          google::protobuf::Timestamp last_stamp = MakeTimestampNow();
          vo->AddImage(gray, event.stamp());
          auto now = MakeTimestampNow();
          LOG(INFO) << google::protobuf::util::TimeUtil::DurationToMilliseconds(
                           now - last_stamp)
                    << " ms";

          int key = cv::waitKey(1) & 0xff;
          if (key == ' ') {
            vo->SetGoal();
          }
          if (key == 'q') {
            break;
          }
        }
        bus_.get_io_service().poll();

      } catch (std::runtime_error& e) {
        LOG(INFO) << e.what();
        bus_.get_io_service().stop();
        bus_.get_io_service().reset();
        break;
      }
    }

    vo->DumpFlowPointsWorld("/tmp/flow_points_world.final.ply");
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
