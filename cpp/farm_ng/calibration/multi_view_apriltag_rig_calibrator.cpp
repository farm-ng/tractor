#include "farm_ng/calibration/multi_view_apriltag_rig_calibrator.h"

#include <ceres/ceres.h>

#include <google/protobuf/util/time_util.h>

#include "farm_ng/blobstore.h"

#include "farm_ng/event_log_reader.h"
#include "farm_ng/ipc.h"
#include "farm_ng/sophus_protobuf.h"

#include "farm_ng/calibration/apriltag.h"

#include "farm_ng/calibration/apriltag_rig_calibrator.h"
#include "farm_ng/calibration/kinematics.h"
#include "farm_ng/calibration/local_parameterization.h"
#include "farm_ng/calibration/pose_utils.h"
#include "farm_ng/calibration/time_series.h"

#include "farm_ng_proto/tractor/v1/apriltag.pb.h"
#include "farm_ng_proto/tractor/v1/capture_video_dataset.pb.h"

namespace farm_ng {

using Sophus::SE3d;

using farm_ng_proto::tractor::v1::CaptureVideoDatasetResult;
using farm_ng_proto::tractor::v1::Event;
using farm_ng_proto::tractor::v1::MultiViewApriltagDetections;

MultiViewApriltagRigModel SolveMultiViewApriltagModel(
    const MultiViewApriltagRigModel& model) {
  return model;
}

std::vector<MultiViewApriltagDetections> LoadMultiViewApriltagDetections(
    const std::string& root_camera_name, const Resource& event_log) {
  EventLogReader log_reader(event_log);

  std::map<std::string, TimeSeries<Event>> apriltag_series;

  while (true) {
    EventPb event;
    try {
      event = log_reader.ReadNext();
    } catch (std::runtime_error& e) {
      break;
    }
    if (event.data().Is<ApriltagDetections>()) {
      apriltag_series[event.name()].insert(event);
    }
  }
  {
    std::stringstream ss;
    ss << "Raw detections\n";
    for (const auto& series : apriltag_series) {
      ss << series.first << " " << series.second.size() << " detections\n";
    }
    LOG(INFO) << ss.str();
  }

  ApriltagsFilter tag_filter;
  std::vector<MultiViewApriltagDetections> mv_detections_series;
  auto time_window =
      google::protobuf::util::TimeUtil::MillisecondsToDuration(1000.0 / 7);

  std::map<std::string, int> detection_counts;
  for (const Event& event : apriltag_series[root_camera_name + "/apriltags"]) {
    ApriltagDetections detections;
    CHECK(event.data().UnpackTo(&detections));
    if (tag_filter.AddApriltags(detections)) {
      MultiViewApriltagDetections mv_detections;
      for (auto name_series : apriltag_series) {
        auto nearest_event =
            name_series.second.FindNearest(event.stamp(), time_window);
        if (nearest_event) {
          CHECK(nearest_event->data().UnpackTo(
              mv_detections.add_detections_per_view()));
          detection_counts[nearest_event->name()]++;
        }
      }
      mv_detections_series.push_back(mv_detections);
    }
  }
  {
    std::stringstream ss;
    ss << "Stable multi-view detections: " << mv_detections_series.size()
       << "\n";
    for (const auto& name_count : detection_counts) {
      ss << name_count.first << " " << name_count.second
         << " stable detections\n";
    }
    LOG(INFO) << ss.str();
  }

  return mv_detections_series;
}

MultiViewApriltagRigModel InitialMultiViewApriltagModelFromConfig(
    const CalibrateMultiViewApriltagRigConfiguration& config) {
  auto dataset_result = ReadProtobufFromResource<CaptureVideoDatasetResult>(
      config.video_dataset());
  MultiViewApriltagRigModel model;

  for (const auto& mv_detections : LoadMultiViewApriltagDetections(
           config.root_camera_name(), dataset_result.dataset())) {
    model.add_multi_view_detections()->CopyFrom(mv_detections);
  }

  return model;
}
}  // namespace farm_ng
