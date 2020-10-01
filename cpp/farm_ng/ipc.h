#ifndef FARM_NG_IPC_H_
#define FARM_NG_IPC_H_
#include <functional>
#include <map>
#include <memory>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>

#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>

#include "farm_ng_proto/tractor/v1/io.pb.h"
#include "farm_ng_proto/tractor/v1/resource.pb.h"

namespace farm_ng {

typedef boost::signals2::signal<void(const farm_ng_proto::tractor::v1::Event&)>
    EventSignal;
typedef std::shared_ptr<EventSignal> EventSignalPtr;

class EventBusImpl;

class EventBus : public boost::asio::io_service::service {
 public:
  EventBus(boost::asio::io_service& io_service);
  ~EventBus();

  // The unique service identifier.
  static boost::asio::io_service::id id;
  // Required by base class.
  void shutdown_service() override;

  EventSignalPtr GetEventSignal() const;

  const std::map<std::string, farm_ng_proto::tractor::v1::Event>& GetState()
      const;

  const std::map<boost::asio::ip::udp::endpoint,
                 farm_ng_proto::tractor::v1::Announce>&
  GetAnnouncements() const;

  void Send(const farm_ng_proto::tractor::v1::Event& event);

  void SetName(const std::string& name);

 private:
  std::unique_ptr<EventBusImpl> impl_;
};

void SetArchivePath(const std::string& name);
boost::filesystem::path GetArchivePath();
boost::filesystem::path GetArchiveRoot();

template <typename ProtobufT>
void WriteProtobufToJsonFile(const boost::filesystem::path& path,
                             const ProtobufT& proto) {
  google::protobuf::util::JsonPrintOptions print_options;
  print_options.add_whitespace = true;
  print_options.always_print_primitive_fields = true;
  std::string json_str;
  google::protobuf::util::MessageToJsonString(proto, &json_str, print_options);
  std::ofstream outf(path.string());
  outf << json_str;
}

template <typename ProtobufT>
void WriteProtobufToBinaryFile(const boost::filesystem::path& path,
                               const ProtobufT& proto) {
  std::string binary_str;
  proto.SerializeToString(&binary_str);
  std::ofstream outf(path.string(), std::ofstream::binary);
  outf << binary_str;
}

// returns a resource that can be written to that will have a unique file
// name, in the active logging directory.
std::pair<farm_ng_proto::tractor::v1::Resource, boost::filesystem::path>
GetUniqueResource(const std::string& prefix, const std::string& ext,
                  const std::string& mime_type);

template <typename ProtobufT>
farm_ng_proto::tractor::v1::Resource WriteProtobufToJsonResource(
    const std::string& prefix, const ProtobufT& message) {
  auto resource_path =
      GetUniqueResource(prefix, "json",
                        "application/json; type=type.googleapis.com/" +
                            ProtobufT::descriptor()->full_name());

  WriteProtobufToJsonFile(resource_path.second, message);
  return resource_path.first;
}

template <typename ProtobufT>
farm_ng_proto::tractor::v1::Resource WriteProtobufToBinaryResource(
    const std::string& prefix, const ProtobufT& message) {
  auto resource_path =
      GetUniqueResource(prefix, "pb",
                        "application/protobuf; type=type.googleapis.com/" +
                            ProtobufT::descriptor()->full_name());

  WriteProtobufToBinaryFile(resource_path.second, message);
  return resource_path.first;
}

google::protobuf::Timestamp MakeTimestampNow();

template <typename T>
farm_ng_proto::tractor::v1::Event MakeEvent(
    std::string name, const T& message,
    const google::protobuf::Timestamp& stamp) {
  farm_ng_proto::tractor::v1::Event event;
  *event.mutable_stamp() = stamp;
  *event.mutable_name() = name;
  event.mutable_data()->PackFrom(message);
  return event;
}

template <typename T>
farm_ng_proto::tractor::v1::Event MakeEvent(std::string name,
                                            const T& message) {
  return MakeEvent(name, message, MakeTimestampNow());
}

inline EventBus& GetEventBus(boost::asio::io_service& io_service,
                             const std::string& service_name) {
  auto& service = boost::asio::use_service<EventBus>(io_service);
  service.SetName(service_name);
  return service;
}

inline void WaitForServices(EventBus& bus,
                            const std::vector<std::string>& service_names) {
  LOG(INFO) << "Waiting for services: ";
  for (const auto& name : service_names) {
    LOG(INFO) << "   " << name;
  }
  bool has_all = false;
  while (!has_all) {
    std::vector<bool> has_service(service_names.size(), false);
    for (const auto& announce : bus.GetAnnouncements()) {
      for (size_t i = 0; i < service_names.size(); ++i) {
        if (announce.second.service() == service_names[i]) {
          has_service[i] = true;
        }
      }
    }
    has_all = true;
    for (auto x : has_service) {
      has_all &= x;
    }
    bus.get_io_service().poll();
  }
}

using farm_ng_proto::tractor::v1::LoggingCommand;
using farm_ng_proto::tractor::v1::LoggingStatus;

inline LoggingStatus WaitForLoggerStatus(
    EventBus& bus, std::function<bool(const LoggingStatus&)> predicate) {
  LoggingStatus status;
  while (true) {
    bus.get_io_service().run_one();
    if (bus.GetState().count("logger/status") &&
        bus.GetState().at("logger/status").data().UnpackTo(&status) &&
        predicate(status)) {
      LOG(INFO) << "Logger status: " << status.ShortDebugString();
      return status;
    }
  }
}

inline LoggingStatus WaitForLoggerStart(EventBus& bus,
                                        const std::string& archive_path) {
  return WaitForLoggerStatus(bus, [archive_path](const LoggingStatus& status) {
    return (status.has_recording() &&
            status.recording().archive_path() == archive_path);
  });
}

inline LoggingStatus WaitForLoggerStop(EventBus& bus) {
  return WaitForLoggerStatus(bus, [](const LoggingStatus& status) {
    return (status.state_case() == LoggingStatus::kStopped);
  });
}

inline LoggingStatus StartLogging(EventBus& bus,
                                  const std::string& archive_path) {
  WaitForLoggerStop(bus);
  LoggingCommand command;
  command.mutable_record_start()->set_archive_path(archive_path);
  bus.Send(farm_ng::MakeEvent("logger/command", command));
  return WaitForLoggerStart(bus, archive_path);
}

inline LoggingStatus StopLogging(EventBus& bus) {
  LoggingCommand command;
  command.mutable_record_stop();
  bus.Send(farm_ng::MakeEvent("logger/command", command));
  return WaitForLoggerStop(bus);
}

}  // namespace farm_ng

#endif
