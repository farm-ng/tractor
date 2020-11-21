#ifndef FARM_NG_IPC_H_
#define FARM_NG_IPC_H_
#include <functional>
#include <map>
#include <memory>

#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>

#include "farm_ng/core/blobstore.h"

#include "farm_ng/core/io.pb.h"
#include "farm_ng/core/resource.pb.h"

namespace farm_ng {
namespace core {

typedef boost::signals2::signal<void(const farm_ng::core::Event&)> EventSignal;
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

  const std::map<std::string, farm_ng::core::Event>& GetState() const;

  const std::map<boost::asio::ip::udp::endpoint, farm_ng::core::Announce>&
  GetAnnouncements() const;

  void AddSubscriptions(const std::vector<Subscription>& subscriptions);
  void AddSubscriptions(const std::vector<std::string>& names);

  void Send(farm_ng::core::Event event);
  // Call async send if io_service is running and calling from a separate
  // thread.
  // TODO revisit thread safe sends.
  void AsyncSend(farm_ng::core::Event event);

  void SetName(const std::string& name);
  std::string GetName();

 private:
  std::unique_ptr<EventBusImpl> impl_;
};

void SetArchivePath(const std::string& name);
boost::filesystem::path GetArchivePath();
boost::filesystem::path GetArchiveRoot();

// returns a resource that can be written to that will have a unique file
// name, in the active logging directory.
std::pair<farm_ng::core::Resource, boost::filesystem::path>
GetUniqueArchiveResource(const std::string& prefix, const std::string& ext,
                         const std::string& mime_type);

// writes a protobuf, as json, to the active logging directory
template <typename ProtobufT>
farm_ng::core::Resource ArchiveProtobufAsJsonResource(
    const std::string& prefix, const ProtobufT& message) {
  auto resource_path =
      GetUniqueArchiveResource(prefix, "json",
                               "application/json; type=type.googleapis.com/" +
                                   ProtobufT::descriptor()->full_name());

  WriteProtobufToJsonFile(resource_path.second, message);
  return resource_path.first;
}

// writes a protobuf, as binary, to the active logging directory
template <typename ProtobufT>
farm_ng::core::Resource ArchiveProtobufAsBinaryResource(
    const std::string& prefix, const ProtobufT& message) {
  auto resource_path = GetUniqueArchiveResource(
      prefix, "pb",
      "application/protobuf; type=type.googleapis.com/" +
          ProtobufT::descriptor()->full_name());

  WriteProtobufToBinaryFile(resource_path.second, message);
  return resource_path.first;
}

google::protobuf::Timestamp MakeTimestampNow();

template <typename T>
farm_ng::core::Event MakeEvent(std::string name, const T& message,
                               const google::protobuf::Timestamp& stamp) {
  farm_ng::core::Event event;
  *event.mutable_stamp() = stamp;
  *event.mutable_name() = name;
  event.mutable_data()->PackFrom(message);
  return event;
}

template <typename T>
farm_ng::core::Event MakeEvent(std::string name, const T& message) {
  return MakeEvent(name, message, MakeTimestampNow());
}

inline EventBus& GetEventBus(boost::asio::io_service& io_service) {
  auto& service = boost::asio::use_service<EventBus>(io_service);
  return service;
}

void WaitForServices(EventBus& bus,
                     const std::vector<std::string>& service_names_in);
LoggingStatus StartLogging(EventBus& bus, const std::string& archive_path);
LoggingStatus StopLogging(EventBus& bus);
void RequestStopLogging(EventBus& bus);

}  // namespace core
}  // namespace farm_ng

#endif
