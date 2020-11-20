#ifndef FARM_NG_FRAME_GRABBER_H_
#define FARM_NG_FRAME_GRABBER_H_
#include <memory>

#include <google/protobuf/timestamp.pb.h>
#include <boost/signals2/signal.hpp>
#include <opencv2/core.hpp>

#include "farm_ng/core/ipc.h"
#include "farm_ng/perception/camera_model.pb.h"
#include "farm_ng/perception/camera_pipeline.pb.h"

namespace farm_ng {
using farm_ng::perception::CameraConfig;
using farm_ng::perception::CameraModel;
struct FrameData {
  CameraConfig config;
  CameraModel camera_model;
  cv::Mat image;
  const google::protobuf::Timestamp& stamp() const { return stamp_; }
  google::protobuf::Timestamp* mutable_stamp() { return &stamp_; }

  google::protobuf::Timestamp stamp_;
};

class FrameGrabber {
 public:
  typedef std::function<std::unique_ptr<FrameGrabber>(EventBus& event_bus,
                                                      CameraConfig config)>
      FrameGrabberFactory;
  static std::unique_ptr<FrameGrabber> MakeFrameGrabber(EventBus& event_bus,
                                                        CameraConfig config);
  static int AddFrameGrabberFactory(const std::string& frame_grabber_name,
                                    FrameGrabberFactory);

  typedef boost::signals2::signal<void(const FrameData&)> Signal;
  virtual ~FrameGrabber();
  virtual const CameraConfig& GetCameraConfig() const = 0;
  virtual const CameraModel& GetCameraModel() const = 0;
  virtual Signal& VisualFrameSignal() = 0;
};

}  // namespace farm_ng
#endif
