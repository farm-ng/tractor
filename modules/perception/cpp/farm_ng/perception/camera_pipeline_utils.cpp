#include "farm_ng/perception/camera_pipeline_utils.h"

#include <glog/logging.h>

#include "farm_ng/perception/camera_pipeline.pb.h"

using farm_ng::core::EventBus;
using farm_ng::core::MakeEvent;

namespace farm_ng {
namespace perception {

void RequestStartCapturing(EventBus& bus,
                           CameraPipelineCommand_RecordStart_Mode mode) {
  CameraPipelineCommand command;
  command.mutable_record_start()->set_mode(mode);
  RequestStartCapturing(bus, command);
}
void RequestStartCapturing(EventBus& bus, CameraPipelineCommand command) {
  LOG(INFO) << "RequestStartCapturing: "
            << MakeEvent("camera_pipeline/command", command).ShortDebugString();
  bus.Send(MakeEvent("camera_pipeline/command", command));
}

void RequestStopCapturing(EventBus& bus) {
  CameraPipelineCommand command;
  command.mutable_record_stop();
  bus.Send(MakeEvent("camera_pipeline/command", command));
}

}  // namespace perception
}  // namespace farm_ng
