#ifndef FARM_NG_PERCEPTION_CREATE_VIDEO_DATASET_H_
#define FARM_NG_PERCEPTION_CREATE_VIDEO_DATASET_H_

#include "farm_ng/core/ipc.h"

#include "farm_ng/perception/create_video_dataset.pb.h"

namespace farm_ng {
namespace perception {

class CreateVideoDatasetProgram {
 public:
  CreateVideoDatasetProgram(core::EventBus& bus,
                            CreateVideoDatasetConfiguration configuration,
                            bool interactive);

  int run();

  void send_status();

  void on_timer(const boost::system::error_code& error);

  bool on_configuration(const core::Event& event);

  void set_configuration(CreateVideoDatasetConfiguration configuration);

  void on_event(const core::Event& event);

 private:
  core::EventBus& bus_;
  boost::asio::deadline_timer timer_;
  CreateVideoDatasetConfiguration configuration_;
  CreateVideoDatasetStatus status_;
};

}  // namespace perception
}  // namespace farm_ng

#endif
