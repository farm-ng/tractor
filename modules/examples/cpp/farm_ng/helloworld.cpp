// A demonstration "HelloWorld" service

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"
#include "farm_ng/examples/helloworld.pb.h"

using farm_ng::core::EventBus;
// using farm_ng::core::LoggingStatus;
using farm_ng::core::MakeEvent;
using farm_ng::core::WaitForServices;
using farm_ng::examples::HelloWorldCommand;
using farm_ng::examples::HelloWorldStatus;
using EventPb = farm_ng::core::Event;

namespace farm_ng {
namespace examples {

// A minimal example of a service that participates on the eventbus
class HelloWorld {
 public:
  HelloWorld(EventBus& bus) : bus_(bus), timer_(bus_.get_io_service()) {
    // Subscribe to eventbus messages with names that match a set of regexes
    bus_.AddSubscriptions(
        {// Matches events whose names include the string 'helloworld'
         bus_.GetName(),
         // Matches logging-related events
         "logger/command", "logger/status"});

    // Invoke the 'on_event' callback when an eventbus message is received
    bus_.GetEventSignal()->connect(
        std::bind(&HelloWorld::on_event, this, std::placeholders::_1));

    // Start this service's periodic loop
    on_timer(boost::system::error_code());
  }

  int run() {
    // Wait for any dependent services to advertise themselves on the eventbus
    // This service depends on the presence of the ipc_logger.
    WaitForServices(bus_, {"ipc_logger"});

    // A synchronous call that requests the ipc_logger to start logging,
    // and returns when it reports that it has.
    StartLogging(bus_, "default");

    // Spin
    while (true) {
      bus_.get_io_service().run_one();
    }

    LOG(INFO) << "Complete.\n";
    return 0;
  }

  // Send a status on the eventbus
  void send_status() {
    HelloWorldStatus status;
    status.set_message("hello, world");
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status));
  }

  // The function invoked by this service's periodic loop
  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }

    // Periodically publish this service's status
    send_status();

    // Schedule the next tick of this periodic loop
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(
        std::bind(&HelloWorld::on_timer, this, std::placeholders::_1));
  }

  // Handle potential HelloWorldCommand messages received from the
  // eventbus
  bool on_command(const EventPb& event) {
    // Attempt to deserialize the eventbus message to a HelloWorldCommand
    HelloWorldCommand command;
    if (!event.data().UnpackTo(&command)) {
      // Silently ignore if it's not a HelloWorldCommand.
      return false;
    }
    LOG(INFO) << command.ShortDebugString();
    return true;
  }

  // Handle messages received from the eventbus
  void on_event(const EventPb& event) {
    // Delegate all messages to a command handler.
    // Messages other than HelloWorldCommands are silently ignored.
    if (on_command(event)) {
      return;
    }
  }

 private:
  // A handle to the eventbus
  EventBus& bus_;

  // The timer that triggers this service's periodic loop.
  boost::asio::deadline_timer timer_;
};

}  // namespace examples
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  farm_ng::examples::HelloWorld service(bus);
  return service.run();
}

// Always invoked by the farm-ng runner, to support graceful shutdown.
void Cleanup(farm_ng::core::EventBus& bus) {
  farm_ng::core::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}

int main(int argc, char* argv[]) {
  // Invoke the farm_ng runner, which provides signal handling,
  // injects the eventbus, etc.
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
