#include "farm_ng/perception/frame_grabber.h"

#include <iostream>

namespace farm_ng {
namespace perception {

namespace {
std::map<std::string, FrameGrabber::FrameGrabberFactory>&
FrameGrabberFactoryRegistry() {
  static std::map<std::string, FrameGrabber::FrameGrabberFactory> registry_;
  return registry_;
}
}  // namespace

std::unique_ptr<FrameGrabber> FrameGrabber::MakeFrameGrabber(
    farm_ng::core::EventBus& event_bus, CameraConfig config) {
  std::string frame_grabber_name = config.frame_grabber_name();
  if (frame_grabber_name.empty()) {
    frame_grabber_name = CameraConfig::Model_Name(config.model());
  }
  auto it = FrameGrabberFactoryRegistry().find(frame_grabber_name);
  CHECK(it != FrameGrabberFactoryRegistry().end())
      << "No FrameGrabberFactory registered for name: " << frame_grabber_name;
  return it->second(event_bus, config);
}
int FrameGrabber::AddFrameGrabberFactory(const std::string& frame_grabber_name,
                                         FrameGrabberFactory factory) {
  if (!FrameGrabberFactoryRegistry()
           .insert(std::make_pair(frame_grabber_name, factory))
           .second) {
    std::cerr << "FrameGrabberFactory Name already registered: "
              << frame_grabber_name << std::endl;
  } else {
    std::cerr << "FramgeGrabberFactory registered for name: "
              << frame_grabber_name << std::endl;
  }
  return 0;
}

FrameGrabber::~FrameGrabber() = default;

}  // namespace perception
}  // namespace farm_ng
