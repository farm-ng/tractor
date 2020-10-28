#ifndef FARM_NG_INTEL_FRAME_GRABBER_H_
#define FARM_NG_INTEL_FRAME_GRABBER_H_
#include <memory>

#include <google/protobuf/timestamp.pb.h>
#include <boost/signals2/signal.hpp>
#include <opencv2/core.hpp>

#include "farm_ng/ipc.h"
#include "farm_ng_proto/tractor/v1/camera_model.pb.h"
#include "farm_ng_proto/tractor/v1/tracking_camera.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::CameraConfig;
using farm_ng_proto::tractor::v1::CameraModel;
struct FrameData {
  CameraConfig config;
  CameraModel camera_model;
  cv::Mat image;
  const google::protobuf::Timestamp& stamp() const { return stamp_; }
  google::protobuf::Timestamp* mutable_stamp() { return &stamp_; }

  google::protobuf::Timestamp stamp_;
};
class IntelFrameGrabber {
 public:
  typedef boost::signals2::signal<void(const FrameData&)> Signal;

  IntelFrameGrabber(EventBus& event_bus, CameraConfig config);
  ~IntelFrameGrabber();
  const CameraConfig& GetCameraConfig() const;
  const CameraModel& GetCameraModel() const;

  Signal& VisualFrameSignal() const;

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace farm_ng
#endif
