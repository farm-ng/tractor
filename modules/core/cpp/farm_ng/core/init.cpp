#include "farm_ng/core/init.h"

#include <stdexcept>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/asio/signal_set.hpp>
#include <boost/exception/all.hpp>
#include <boost/filesystem.hpp>
#include <boost/stacktrace.hpp>
#include <iostream>
#include <memory>

#include "farm_ng/core/ipc.h"

typedef boost::error_info<struct tag_stacktrace, boost::stacktrace::stacktrace>
    traced;

namespace farm_ng {
namespace core {

namespace {
boost::asio::io_service& _get_io_service() {
  static boost::asio::io_service io_service;
  return io_service;
}

boost::asio::signal_set& _get_signal_set() {
  static boost::asio::signal_set signals(_get_io_service(), SIGTERM, SIGINT);
  return signals;
}

void (*_g_cleanup_func)(EventBus&) = nullptr;

void _signal_handler(const boost::system::error_code& error,
                     int signal_number) {
  std::cout << "Received (error, signal) " << error << " , " << signal_number
            << std::endl;
  // std::quick_exit(signal_number);
  _get_io_service().stop();
  _get_signal_set().async_wait(&_signal_handler);

  throw std::runtime_error(std::to_string(signal_number));
}
}  // namespace
void GlogFailureFunction() {
  std::cerr << boost::stacktrace::stacktrace() << std::endl;
  if (_g_cleanup_func != nullptr) {
    _g_cleanup_func(GetEventBus(_get_io_service()));
  }
  std::quick_exit(-1);
}

int Main(int argc, char** argv, int (*main_func)(EventBus&),
         void (*cleanup_func)(EventBus&)) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;

  std::string filename = boost::filesystem::path(argv[0]).filename().string();
  google::InitGoogleLogging(filename.c_str());
  google::InstallFailureFunction(&GlogFailureFunction);
  google::InstallFailureSignalHandler();

  _get_signal_set().async_wait(&_signal_handler);

  EventBus& bus = GetEventBus(_get_io_service());
  bus.SetName(filename);
  _g_cleanup_func = cleanup_func;

  std::shared_ptr<void (*)(EventBus&)> callme(&_g_cleanup_func, [](auto p) {
    (*p)(GetEventBus(_get_io_service()));
    // run any jobs scheduled by cleanup function on the io_service.
    //_get_io_service().reset();
    //_get_io_service().poll();
  });

  try {
    return main_func(bus);
  } catch (std::exception& e) {
    LOG(ERROR) << e.what();

    const boost::stacktrace::stacktrace* st = boost::get_error_info<traced>(e);
    if (st) {
      LOG(ERROR) << *st;
    }
    return 1;
  } catch (...) {
    LOG(ERROR) << "unkown exception thrown.";
    return 1;
  }
}

}  // namespace core
}  // namespace farm_ng
