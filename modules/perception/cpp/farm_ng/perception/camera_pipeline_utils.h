#ifndef FARM_NG_TRACKING_CAMERA_UTILS_H_
#define FARM_NG_TRACKING_CAMERA_UTILS_H_

#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/camera_pipeline.pb.h"

namespace farm_ng {
namespace perception {

void RequestStartCapturing(farm_ng::core::EventBus& bus,
                           CameraPipelineCommand_RecordStart_Mode mode);
void RequestStartCapturing(farm_ng::core::EventBus& bus,
                           CameraPipelineCommand command);
void RequestStopCapturing(farm_ng::core::EventBus& bus);

}  // namespace perception
}  // namespace farm_ng

#endif
