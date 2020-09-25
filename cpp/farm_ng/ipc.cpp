#include "farm_ng/ipc.h"

#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/time_util.h>

#include <glog/logging.h>

#include "farm_ng_proto/tractor/v1/io.pb.h"

namespace farm_ng {
using farm_ng_proto::tractor::v1::LoggingCommand;
namespace {

class ArchiveManager {
 public:
  ArchiveManager() : resource_uuids_(0) {}
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
  boost::filesystem::path root_ = "/tmp/farm-ng-log/";
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

    farm_ng_proto::tractor::v1::Announce announce;
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
    announcements_[sender_endpoint_] = announce;

    socket_.async_receive_from(
        boost::asio::buffer(data_, max_datagram_size), sender_endpoint_,
        std::bind(&receiver::handle_receive_from, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

  const std::map<boost::asio::ip::udp::endpoint,
                 farm_ng_proto::tractor::v1::Announce>&
  announcements() const {
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
  std::map<boost::asio::ip::udp::endpoint, farm_ng_proto::tractor::v1::Announce>
      announcements_;

  char data_[max_datagram_size];
};

}  // namespace
typedef boost::signals2::signal<void(const farm_ng_proto::tractor::v1::Event&)>
    EventSignal;
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
    farm_ng_proto::tractor::v1::Announce announce;
    // For now, only announce our local address
    // announce.set_host(endpoint.address().to_string());
    announce.set_host("127.0.0.1");
    announce.set_port(endpoint.port());
    announce.set_service(service_name_);

    *announce.mutable_stamp() = MakeTimestampNow();
    announce.SerializeToString(&announce_message_);

    recv_.clear_stale_announcements();

    socket_.send_to(boost::asio::buffer(announce_message_), announce_endpoint_);
  }

  void handle_receive_from(const boost::system::error_code& error,
                           size_t bytes_recvd) {
    if (!error) {
      farm_ng_proto::tractor::v1::Event event;
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

  void send_event(const farm_ng_proto::tractor::v1::Event& event) {
    event.SerializeToString(&event_message_);
    CHECK_LT(int(event_message_.size()), max_datagram_size)
        << "Event is too big, doesn't fit in one udp packet.";
    for (const auto& it : recv_.announcements()) {
      boost::asio::ip::udp::endpoint ep(
          boost::asio::ip::address::from_string(it.second.host()),
          it.second.port());
      socket_.send_to(boost::asio::buffer(event_message_), ep);
    }
  }

  void set_name(const std::string& name) { service_name_ = name; }

  boost::asio::io_service& io_service_;

  receiver recv_;

 private:
  boost::asio::ip::udp::socket socket_;
  boost::asio::deadline_timer announce_timer_;

  boost::asio::ip::udp::endpoint announce_endpoint_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  char data_[max_datagram_size];
  std::string announce_message_;
  std::string event_message_;
  std::string service_name_ = "unknown [cpp-ipc]";

 public:
  std::map<std::string, farm_ng_proto::tractor::v1::Event> state_;
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

const std::map<std::string, farm_ng_proto::tractor::v1::Event>&
EventBus::GetState() const {
  return impl_->state_;
}
const std::map<boost::asio::ip::udp::endpoint,
               farm_ng_proto::tractor::v1::Announce>&
EventBus::GetAnnouncements() const {
  return impl_->recv_.announcements();
}
void EventBus::Send(const farm_ng_proto::tractor::v1::Event& event) {
  impl_->io_service_.post([this, event]() { impl_->send_event(event); });
}
void EventBus::SetName(const std::string& name) { impl_->set_name(name); }

void SetArchivePath(const std::string& name) { get_archive().SetPath(name); }

boost::filesystem::path GetArchivePath() { return get_archive().path(); }
boost::filesystem::path GetArchiveRoot() { return get_archive().root(); }

std::pair<farm_ng_proto::tractor::v1::Resource, boost::filesystem::path>
GetUniqueResource(const std::string& prefix, const std::string& ext,
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

}  // namespace farm_ng
