#include "farm_ng/perception/camera_pipeline.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

void Cleanup(farm_ng::core::EventBus& bus) {}

int Main(farm_ng::core::EventBus& bus) {
  try {
    farm_ng::perception::CameraPipelineClient client(bus);
    bus.get_io_service().run();
  } catch (...) {
  }
  return 0;
}

int main(int argc, char* argv[]) {
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
