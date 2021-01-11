// A demonstration "Countdown" program

#include <iostream>
#include <optional>
#include <sstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/countdown.pb.h"
#include "farm_ng/core/init.h"
#include "farm_ng/core/ipc.h"

DEFINE_bool(interactive, false, "Receive program args via eventbus");
DEFINE_string(name, "default", "A name for the result file");
DEFINE_uint64(start, 10, "The starting number for the countdown");

using farm_ng::core::CountdownConfiguration;
using farm_ng::core::CountdownResult;
using farm_ng::core::CountdownStatus;
using EventPb = farm_ng::core::Event;

namespace farm_ng {
namespace core {

// Count down from N to 0, outputting each value to a result file in the
// blobstore.
class CountdownProgram {
 public:
  CountdownProgram(EventBus& bus, CountdownConfiguration configuration,
                   bool interactive)
      : bus_(bus), timer_(bus_.get_io_service()) {
    // A ConfigurationStatus message is periodically sent on the eventbus.
    // If running in interactive mode, update this status to indicate that
    // user input is required.
    if (interactive) {
      status_.mutable_input_required_configuration()->CopyFrom(configuration);
    } else {
      set_configuration(configuration);
    }

    // Subscribe to eventbus messages with names that match a set of regexes
    bus_.AddSubscriptions(
        {// Matches events whose names include the string 'countdown_program'
         bus_.GetName(),
         // Matches logging-related events
         "logger/command", "logger/status"});

    // Invoke the 'on_event' callback when an eventbus message is received
    bus_.GetEventSignal()->connect(
        std::bind(&CountdownProgram::on_event, this, std::placeholders::_1));

    // Start this program's periodic loop
    on_timer(boost::system::error_code());
  }

  int run() {
    // If running in interactive mode, loop until a configuration is received on
    // the eventbus.
    while (status_.has_input_required_configuration()) {
      bus_.get_io_service().run_one();
    }

    // Wait for any dependent services to advertise themselves on the eventbus
    // This program depends on the presence of the ipc_logger.
    WaitForServices(bus_, {"ipc_logger"});

    // A synchronous call that requests the ipc_logger to start logging,
    // and returns when it reports that it has.
    LoggingStatus log = StartLogging(bus_, configuration_.name());

    // Instantiate a result and populate it with the start time of this program.
    // The rest of the fields will be populated at program completion.
    CountdownResult result;
    result.mutable_stamp_begin()->CopyFrom(MakeTimestampNow());

    // Start the countdown and spin
    countdown_started_ = true;
    while (status_.count() > 0) {
      bus_.get_io_service().run_one();
    }

    // Populate the result with the configuration used by this program
    result.mutable_configuration()->CopyFrom(configuration_);
    // Populate the result with the end time of this program
    result.mutable_stamp_end()->CopyFrom(MakeTimestampNow());
    // Populate the result with the path to the recorded log
    result.mutable_dataset()->set_path(log.recording().archive_path());
    // Send one last status message on the eventbus
    send_status();

    LOG(INFO) << "Complete:\n" << status_.DebugString();
    return 0;
  }

  // Send this program's status on the eventbus
  void send_status() {
    LOG(INFO) << "Status:\n" << status_.DebugString();
    bus_.Send(MakeEvent(bus_.GetName() + "/status", status_));
  }

  // The function invoked by this program's periodic loop
  void on_timer(const boost::system::error_code& error) {
    if (error) {
      LOG(WARNING) << "timer error: " << __PRETTY_FUNCTION__ << error;
      return;
    }

    // Count down
    if (countdown_started_) {
      status_.set_count(status_.count() - 1);
    }

    // Periodically publish this program's status
    send_status();

    // Schedule the next tick of this periodic loop
    timer_.expires_from_now(boost::posix_time::millisec(1000));
    timer_.async_wait(
        std::bind(&CountdownProgram::on_timer, this, std::placeholders::_1));
  }

  // Handle potential CountdownConfiguration messages received from the
  // eventbus
  bool on_configuration(const EventPb& event) {
    // Attempt to deserialize the eventbus message to a CountdownConfiguration
    CountdownConfiguration configuration;
    if (!event.data().UnpackTo(&configuration)) {
      // Silently ignore if it's not a CountdownConfiguration.
      return false;
    }
    LOG(INFO) << configuration.ShortDebugString();
    set_configuration(configuration);
    return true;
  }

  // Update this program's configuration
  void set_configuration(CountdownConfiguration configuration) {
    configuration_ = configuration;

    // Initialize the counter
    status_.set_count(configuration.start());

    // No longer advertise to the eventbus that configuration is required
    status_.clear_input_required_configuration();
    send_status();
  }

  // Handle messages received from the eventbus
  void on_event(const EventPb& event) {
    // Delegate all messages to a configuration handler.
    // Messages other than configuration messages are silently ignored.
    if (on_configuration(event)) {
      return;
    }
  }

 private:
  // A handle to the eventbus
  EventBus& bus_;

  // The timer that triggers this program's periodic loop.
  boost::asio::deadline_timer timer_;

  // This program's configuration
  CountdownConfiguration configuration_;

  // This program's current status
  CountdownStatus status_;

  // Whether the countdown has started
  std::atomic<bool> countdown_started_{false};
};

}  // namespace core
}  // namespace farm_ng

int Main(farm_ng::core::EventBus& bus) {
  // Populate the program configuration with any relevant command-line flags
  farm_ng::core::CountdownConfiguration config;
  config.set_name(FLAGS_name);
  config.set_start(FLAGS_start);

  // Run the program
  farm_ng::core::CountdownProgram program(bus, config, FLAGS_interactive);
  return program.run();
}

// This function is invoked by signal handlers, whether the program exited
// successfully or not.
void Cleanup(farm_ng::core::EventBus& bus) {
  farm_ng::core::RequestStopLogging(bus);
  LOG(INFO) << "Requested Stop logging";
}

int main(int argc, char* argv[]) {
  // Invoke the farm_ng program runner, which provides signal handling,
  // injects the eventbus, etc.
  return farm_ng::core::Main(argc, argv, &Main, &Cleanup);
}
