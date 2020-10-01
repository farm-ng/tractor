#include <gflags/gflags.h>

#include "farm_ng/ipc.h"

int main(int argc, char* argv[]) {
  // Initialize Google's logging library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  boost::asio::io_service io_service;
  farm_ng::EventBus& bus = farm_ng::GetEventBus(io_service, "servo_controller");

  farm_ng::WaitForServices(bus, {"tractor", "tracking_camera"});

  io_service.run();

  return 0;
}
