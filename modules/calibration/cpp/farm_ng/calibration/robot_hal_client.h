#ifndef FARM_NG_CALIBRATION_ROBOT_HAL_CLIENT_H_
#define FARM_NG_CALIBRATION_ROBOT_HAL_CLIENT_H_

#include "farm_ng/calibration/robot_hal.grpc.pb.h"
namespace farm_ng::calibration {

class RobotHalClient {
 public:
  explicit RobotHalClient(const std::string& hal_service_address);

  std::tuple<grpc::Status, CapturePoseResponse> CapturePoseSync(
      const CapturePoseRequest& request);

  std::tuple<grpc::Status, CalibrationResultResponse> CalibrationResultSync(
      const CalibrationResultRequest& request);

  std::tuple<grpc::Status, CalibratedCaptureResponse> CalibratedCaptureSync(
      const CalibratedCaptureRequest& request);

  std::tuple<grpc::Status, ApriltagRigPoseEstimateResponse>
  ApriltagRigPoseEstimateSync(const ApriltagRigPoseEstimateRequest& request);

 private:
  std::unique_ptr<RobotHALService::Stub> stub_;
};
}  // namespace farm_ng::calibration
#endif
