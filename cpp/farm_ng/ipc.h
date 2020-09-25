#ifndef FARM_NG_IPC_H_
#define FARM_NG_IPC_H_
#include <functional>
#include <map>
#include <memory>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>

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

}  // namespace farm_ng

#endif
