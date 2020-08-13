#include "farm_ng/event_log.h"
#include "farm_ng/ipc.h"

#include <iostream>

namespace farm_ng {

class IpcLogger {
 public:
  IpcLogger(boost::asio::io_service& io_service)
      : bus_(GetEventBus(io_service)),
        log_writer_("/tmp/farm-ng-event.log"),
        log_timer_(io_service),
        announce_timer_(io_service) {
    bus_.GetEventSignal()->connect(
        std::bind(&IpcLogger::on_event, this, std::placeholders::_1));
    log_state(boost::system::error_code());
    log_announce(boost::system::error_code());
  }
  void on_event(const farm_ng_proto::tractor::v1::Event& event) {
    log_writer_.Write(event);
  }
  void log_announce(const boost::system::error_code& error) {
    if (error) {
      std::cerr << "announce timer error: " << __PRETTY_FUNCTION__ << error
                << std::endl;
      return;
    }
    announce_timer_.expires_from_now(boost::posix_time::seconds(10));
    announce_timer_.async_wait(
        std::bind(&IpcLogger::log_announce, this, std::placeholders::_1));

    for (const auto& it : bus_.GetAnnouncements()) {
      std::cout << it.second.ShortDebugString() << std::endl;
    }
  }
  void log_state(const boost::system::error_code& error) {
    if (error) {
      std::cerr << "log timer error: " << __PRETTY_FUNCTION__ << error
                << std::endl;
      return;
    }
    log_timer_.expires_from_now(boost::posix_time::seconds(1));
    log_timer_.async_wait(
        std::bind(&IpcLogger::log_state, this, std::placeholders::_1));

    for (const auto& it : bus_.GetState()) {
      std::cout << it.first << " : " << it.second.ShortDebugString()
                << std::endl;
    }
  }

 private:
  EventBus& bus_;
  EventLogWriter log_writer_;
  boost::asio::deadline_timer log_timer_;
  boost::asio::deadline_timer announce_timer_;
};

}  // namespace farm_ng

int main(int argc, char* argv[]) {
  try {
    boost::asio::io_service io_service;
    farm_ng::IpcLogger logger(io_service);
    io_service.run();
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
