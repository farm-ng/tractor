#include "farm_ng/calibration/robot_hal_client.h"

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

namespace farm_ng::calibration {

namespace {

std::shared_ptr<grpc::Channel> MakeInsecureChannel(
    const std::string& hal_service_address) {
  grpc::ChannelArguments ch_args;
  ch_args.SetMaxReceiveMessageSize(100000000);
  ch_args.SetMaxSendMessageSize(100000000);
  auto credentials = grpc::InsecureChannelCredentials();
  auto channel =
      grpc::CreateCustomChannel(hal_service_address, credentials, ch_args);
  return channel;
}
}  // namespace
RobotHalClient::RobotHalClient(const std::string& hal_service_address)
    : stub_(
          RobotHALService::NewStub(MakeInsecureChannel(hal_service_address))) {}

std::tuple<grpc::Status, CapturePoseResponse> RobotHalClient::CapturePoseSync(
    const CapturePoseRequest& request) {
  grpc::ClientContext context;

  std::shared_ptr<
      grpc::ClientReaderWriter<CapturePoseRequest, CapturePoseResponse>>
      stream(stub_->CapturePose(&context));

  stream->Write(request);
  stream->WritesDone();

  // Block waiting for response
  CapturePoseResponse response;
  stream->Read(&response);
  return {stream->Finish(), response};
}

std::tuple<grpc::Status, CalibrationResultResponse>
RobotHalClient::CalibrationResultSync(const CalibrationResultRequest& request) {
  CalibrationResultResponse response;
  grpc::ClientContext context;
  return {stub_->CalibrationResult(&context, request, &response), response};
}

std::tuple<grpc::Status, CalibratedCaptureResponse>
RobotHalClient::CalibratedCaptureSync(const CalibratedCaptureRequest& request) {
  CalibratedCaptureResponse response;
  grpc::ClientContext context;
  return {stub_->CalibratedCapture(&context, request, &response), response};
}

std::tuple<grpc::Status, ApriltagRigPoseEstimateResponse>
RobotHalClient::ApriltagRigPoseEstimateSync(
    const ApriltagRigPoseEstimateRequest& request) {
  ApriltagRigPoseEstimateResponse response;
  grpc::ClientContext context;
  return {stub_->ApriltagRigPoseEstimate(&context, request, &response),
          response};
}

}  // namespace farm_ng::calibration
