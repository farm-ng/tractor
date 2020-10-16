#include "farm_ng/ipc.h"

#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <mutex>
#include <regex>
#include <string>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/time_util.h>

#include <glog/logging.h>

#include "farm_ng/blobstore.h"
#include "farm_ng_proto/tractor/v1/io.pb.h"
#include "farm_ng_proto/tractor/v1/tracking_camera.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::Announce;
using farm_ng_proto::tractor::v1::Event;
using farm_ng_proto::tractor::v1::BUCKET_LOGS;
using farm_ng_proto::tractor::v1::LoggingCommand;
using farm_ng_proto::tractor::v1::LoggingStatus;
using farm_ng_proto::tractor::v1::Subscription;
using farm_ng_proto::tractor::v1::TrackingCameraCommand;
using farm_ng_proto::tractor::v1::TrackingCameraCommand_RecordStart_Mode;
namespace {

class ArchiveManager {
 public:
  ArchiveManager() : resource_uuids_(0) { root_ = GetBlobstoreRoot(); }
  boost::filesystem::path path() const {
    std::lock_guard<std::mutex> lock(path_mtx_);
    return path_;
  }

  void SetPath(const std::string& name) {
    std::lock_guard<std::mutex> lock(path_mtx_);
    path_ = name;
  }
  boost::filesystem::path root() const { return root_; }

  uint64_t NewResourceId() { return resource_uuids_++; }

 private:
  boost::filesystem::path root_;
  mutable std::mutex path_mtx_;
  boost::filesystem::path path_ = "default";
  std::atomic<uint64_t> resource_uuids_;
};

ArchiveManager& get_archive() {
  static ArchiveManager manager_;
  return manager_;
}

enum { max_datagram_size = 65507 };
const short multicast_port = 10000;
std::string g_multicast_address = "239.20.20.21";

template <typename Duration>
using sys_time = std::chrono::time_point<std::chrono::system_clock, Duration>;

template <typename Duration>
google::protobuf::Timestamp MakeTimestamp(sys_time<Duration> const& tp) {
  long long nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        tp.time_since_epoch())
                        .count();
  google::protobuf::Timestamp stamp;
  stamp.set_seconds(nanos / 1000000000);
  stamp.set_nanos(nanos % 1000000000);
  return stamp;
}

class receiver {
 public:
  receiver(boost::asio::io_service& io_service,
           const boost::asio::ip::address& listen_address,
           const boost::asio::ip::address& multicast_address)
      : socket_(io_service) {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(listen_address,
                                                   multicast_port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    // Join the multicast group.
    socket_.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));

    socket_.async_receive_from(
        boost::asio::buffer(data_, max_datagram_size), sender_endpoint_,
        std::bind(&receiver::handle_receive_from, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

  void handle_receive_from(const boost::system::error_code& error,
                           size_t bytes_recvd) {
    if (error) {
      std::cerr << "handle_receive_from error: " << error << std::endl;
      return;
    }

    // Ignore self-announcements
    if (sender_endpoint_.port() == multicast_port) {
      return;
    }

    // Ignore non-local announcements
    // TODO: Implement this

    Announce announce;
    announce.ParseFromArray(static_cast<const void*>(data_), bytes_recvd);
    *announce.mutable_recv_stamp() = MakeTimestampNow();

    // Ignore faulty announcements
    if (announce.host() != "127.0.0.1" ||
        announce.port() != sender_endpoint_.port()) {
      std::cerr << "announcement does not match sender... rejecting: "
                << announce.host() << ":" << announce.port() << std::endl;
      return;
    }

    // Store the announcement
    announcements_[boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(announce.host()),
        announce.port())] = announce;

    socket_.async_receive_from(
        boost::asio::buffer(data_, max_datagram_size), sender_endpoint_,
        std::bind(&receiver::handle_receive_from, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

  const std::map<boost::asio::ip::udp::endpoint, Announce>& announcements()
      const {
    return announcements_;
  }

  void clear_stale_announcements() {
    auto now = google::protobuf::util::TimeUtil::TimestampToSeconds(
        MakeTimestampNow());
    std::vector<boost::asio::ip::udp::endpoint> keys_to_remove;
    for (const auto& it : announcements_) {
      auto recv_time = google::protobuf::util::TimeUtil::TimestampToSeconds(
          it.second.recv_stamp());
      if (now - recv_time > 10) {
        keys_to_remove.push_back(it.first);
      }
    }
    for (const auto& k : keys_to_remove) {
      // std::cout << "Removing stale service." << k << std::endl;
      announcements_.erase(k);
    }
  }

 private:
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::map<boost::asio::ip::udp::endpoint, Announce> announcements_;

  char data_[max_datagram_size];
};

// Memoized regex compilation
std::unordered_map<std::string, std::regex> compiled_;
std::regex compile_regex(const std::string& s) {
  if (compiled_.find(s) == compiled_.end()) {
    compiled_[s] = std::regex(s);
  }
  return compiled_.at(s);
}

bool is_recipient(const Announce& announce, const Event& event) {
  return std::any_of(
      announce.subscriptions().begin(), announce.subscriptions().end(),
      [event](const Subscription& subscription) {
        std::smatch match;
        return std::regex_search(event.name(), match,
                                 compile_regex(subscription.name()));
      });
}

}  // namespace
typedef boost::signals2::signal<void(const Event&)> EventSignal;
typedef std::shared_ptr<EventSignal> EventSignalPtr;

class EventBusImpl {
 public:
  EventBusImpl(boost::asio::io_service& io_service,
               const boost::asio::ip::address& listen_address,
               const boost::asio::ip::address& multicast_address)
      : io_service_(io_service),
        recv_(io_service, listen_address, multicast_address),
        socket_(io_service),
        announce_timer_(io_service),
        announce_endpoint_(multicast_address, multicast_port),
        signal_(new EventSignal) {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(listen_address, 0);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    socket_.async_receive_from(
        boost::asio::buffer(data_, max_datagram_size), sender_endpoint_,
        std::bind(&EventBusImpl::handle_receive_from, this,
                  std::placeholders::_1, std::placeholders::_2));

    send_announce(boost::system::error_code());
  }

  void send_announce(const boost::system::error_code& error) {
    if (error) {
      std::cerr << "announce timer error: " << error << std::endl;
      return;
    }
    announce_timer_.expires_from_now(boost::posix_time::seconds(1));
    announce_timer_.async_wait(
        std::bind(&EventBusImpl::send_announce, this, std::placeholders::_1));

    auto endpoint = socket_.local_endpoint();
    Announce announce;
    // For now, only announce our local address
    // announce.set_host(endpoint.address().to_string());
    announce.set_host("127.0.0.1");
    announce.set_port(endpoint.port());
    announce.set_service(service_name_);
    *announce.mutable_subscriptions() = {subscriptions_.begin(),
                                         subscriptions_.end()};

    *announce.mutable_stamp() = MakeTimestampNow();
    announce.SerializeToString(&announce_message_);

    recv_.clear_stale_announcements();

    socket_.send_to(boost::asio::buffer(announce_message_), announce_endpoint_);
  }

  void handle_receive_from(const boost::system::error_code& error,
                           size_t bytes_recvd) {
    if (!error) {
      Event event;
      CHECK(event.ParseFromArray(static_cast<const void*>(data_), bytes_recvd));

      if (event.data().type_url() ==
          "type.googleapis.com/" + LoggingCommand::descriptor()->full_name()) {
        LoggingCommand command;
        CHECK(event.data().UnpackTo(&command));
        if (command.has_record_start()) {
          SetArchivePath(command.record_start().archive_path());
        } else {
          SetArchivePath("default");
        }
      }

      state_[event.name()] = event;
      (*signal_)(event);

      socket_.async_receive_from(
          boost::asio::buffer(data_, max_datagram_size), sender_endpoint_,
          std::bind(&EventBusImpl::handle_receive_from, this,
                    std::placeholders::_1, std::placeholders::_2));
    }
  }

  void send_event(const Event& event) {
    auto recipient_list = recipients(event);
    if (recipient_list.empty()) {
      return;
    }

    event.SerializeToString(&event_message_);
    CHECK_LT(int(event_message_.size()), max_datagram_size)
        << "Event is too big, doesn't fit in one udp packet.";
    for (const auto& recipient : recipient_list) {
      socket_.send_to(boost::asio::buffer(event_message_), recipient);
    }
  }

  void add_subscriptions(const std::vector<Subscription>& subscriptions) {
    subscriptions_.insert(subscriptions_.end(), subscriptions.begin(),
                          subscriptions.end());
  }

  const std::vector<Subscription>& subscriptions() const {
    return subscriptions_;
  }

  void set_name(const std::string& name) { service_name_ = name; }
  std::string get_name() { return service_name_; }

  boost::asio::io_service& io_service_;

  receiver recv_;

 private:
  const std::vector<boost::asio::ip::udp::endpoint> recipients(
      const Event& event) {
    std::vector<boost::asio::ip::udp::endpoint> result;
    for (auto& it : recv_.announcements()) {
      if (is_recipient(it.second, event)) {
        result.push_back(it.first);
      }
    }
    return result;
  }

  boost::asio::ip::udp::socket socket_;
  boost::asio::deadline_timer announce_timer_;

  boost::asio::ip::udp::endpoint announce_endpoint_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  char data_[max_datagram_size];
  std::string announce_message_;
  std::string event_message_;
  std::string service_name_ = "unknown [cpp-ipc]";
  std::vector<Subscription> subscriptions_;

 public:
  std::map<std::string, Event> state_;
  EventSignalPtr signal_;
};

boost::asio::io_service::id EventBus::id;

void EventBus::shutdown_service() {}

EventBus::EventBus(boost::asio::io_service& io_service)
    : boost::asio::io_service::service(io_service),
      impl_(new EventBusImpl(
          io_service, boost::asio::ip::address::from_string("0.0.0.0"),
          boost::asio::ip::address::from_string(g_multicast_address))) {}

EventBus::~EventBus() { impl_.reset(nullptr); }

EventSignalPtr EventBus::GetEventSignal() const { return impl_->signal_; }

const std::map<std::string, Event>& EventBus::GetState() const {
  if (impl_->subscriptions().empty()) {
    LOG(WARNING) << "This EventBus has no subscriptions registered";
  }
  return impl_->state_;
}
const std::map<boost::asio::ip::udp::endpoint, Announce>&
EventBus::GetAnnouncements() const {
  return impl_->recv_.announcements();
}
void EventBus::AddSubscriptions(
    const std::vector<Subscription>& subscriptions) {
  return impl_->add_subscriptions(subscriptions);
}
void EventBus::AddSubscriptions(const std::vector<std::string>& names) {
  std::vector<Subscription> subscriptions;
  std::transform(names.begin(), names.end(), std::back_inserter(subscriptions),
                 [](const std::string& name) {
                   Subscription subscription;
                   subscription.set_name(name);
                   return subscription;
                 });
  return AddSubscriptions(subscriptions);
}
void EventBus::Send(const Event& event) { impl_->send_event(event); }
void EventBus::SetName(const std::string& name) { impl_->set_name(name); }
std::string EventBus::GetName() { return impl_->get_name(); }

void SetArchivePath(const std::string& name) {
  LOG(INFO) << "Setting archive path to: " << name;
  get_archive().SetPath(name);
}

boost::filesystem::path GetArchivePath() { return get_archive().path(); }
boost::filesystem::path GetArchiveRoot() { return get_archive().root(); }

std::pair<farm_ng_proto::tractor::v1::Resource, boost::filesystem::path>
GetUniqueArchiveResource(const std::string& prefix, const std::string& ext,
                         const std::string& content_type) {
  farm_ng_proto::tractor::v1::Resource resource;
  resource.set_content_type(content_type);

  pid_t pid = getpid();
  uint64_t id = get_archive().NewResourceId();
  char buffer[1024];
  if (std::snprintf(buffer, sizeof(buffer), "%s-%05d-%05ld.%s", prefix.c_str(),
                    pid, id, ext.c_str()) > int(sizeof(buffer) - 1)) {
    throw std::runtime_error("path is too long.");
  }
  auto path = GetArchivePath() / buffer;

  resource.set_path(path.string());
  boost::filesystem::path writable_path = GetArchiveRoot() / path;
  if (!boost::filesystem::exists(writable_path.parent_path())) {
    if (!boost::filesystem::create_directories(writable_path.parent_path())) {
      throw std::runtime_error(std::string("Could not create directory: ") +
                               writable_path.parent_path().string());
    }
  }

  return std::make_pair(resource, writable_path);
}

google::protobuf::Timestamp MakeTimestampNow() {
  return MakeTimestamp(std::chrono::system_clock::now());
}

void WaitForServices(EventBus& bus,
                     const std::vector<std::string>& service_names_in) {
  std::vector<std::string> service_names(service_names_in.begin(),
                                         service_names_in.end());

  // Wait on ourself too
  service_names.push_back(bus.GetName());

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
  LOG(INFO) << "Found all services.";
}

LoggingStatus WaitForLoggerStatus(
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

LoggingStatus WaitForLoggerStart(EventBus& bus,
                                 const std::string& archive_path) {
  return WaitForLoggerStatus(bus, [archive_path](const LoggingStatus& status) {
    LOG(INFO)
        << "Waiting for logger status.recording.archive_path starting with:"
        << archive_path << " status:" << status.ShortDebugString();
    return (status.has_recording() &&
            status.recording().archive_path().rfind(archive_path) == 0);
  });
}

LoggingStatus WaitForLoggerStop(EventBus& bus) {
  return WaitForLoggerStatus(bus, [](const LoggingStatus& status) {
    return (status.state_case() == LoggingStatus::kStopped);
  });
}

LoggingStatus StartLogging(EventBus& bus, const std::string& archive_path) {
  WaitForLoggerStop(bus);
  LoggingCommand command;
  std::string full_archive_path = (GetBucketRelativePath(BUCKET_LOGS) /
                                   boost::filesystem::path(archive_path))
                                      .string();
  command.mutable_record_start()->set_archive_path(full_archive_path);
  LOG(INFO) << "start: " << command.ShortDebugString();
  bus.Send(farm_ng::MakeEvent("logger/command", command));
  return WaitForLoggerStart(bus, full_archive_path);
}

LoggingStatus StopLogging(EventBus& bus) {
  RequestStopLogging(bus);
  return WaitForLoggerStop(bus);
}

void RequestStopLogging(EventBus& bus) {
  LoggingCommand command;
  command.mutable_record_stop();
  bus.Send(farm_ng::MakeEvent("logger/command", command));
}

void RequestStartCapturing(EventBus& bus,
                           TrackingCameraCommand_RecordStart_Mode mode) {
  TrackingCameraCommand command;
  command.mutable_record_start()->set_mode(mode);
  LOG(INFO) << "RequestStartCapturing: "
            << farm_ng::MakeEvent("tracking_camera/command", command)
                   .ShortDebugString();
  bus.Send(farm_ng::MakeEvent("tracking_camera/command", command));
}

void RequestStopCapturing(EventBus& bus) {
  TrackingCameraCommand command;
  command.mutable_record_stop();
  bus.Send(farm_ng::MakeEvent("tracking_camera/command", command));
}

}  // namespace farm_ng
