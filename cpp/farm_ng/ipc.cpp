#include "farm_ng/ipc.h"



#include <iostream>
#include <string>
#include <functional>
#include <chrono>

#include <boost/asio.hpp>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/time_util.h>

#include "farm_ng_proto/tractor/v1/io.pb.h"

namespace farm_ng {
  namespace {
const short multicast_port = 10000;
std::string g_multicast_address = "239.20.20.21";

template <typename Duration>
using sys_time = std::chrono::time_point<std::chrono::system_clock, Duration>;

template<typename Duration>
google::protobuf::Timestamp MakeTimestamp(sys_time<Duration> const& tp) {
  long long nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
  google::protobuf::Timestamp stamp;
  stamp.set_seconds(nanos / 1000000000);
  stamp.set_nanos(nanos % 1000000000);
  return stamp;
}

google::protobuf::Timestamp MakeTimestampNow() {
  return MakeTimestamp(std::chrono::system_clock::now());
}

class receiver
{
public:
  receiver(boost::asio::io_service& io_service,
      const boost::asio::ip::address& listen_address,
      const boost::asio::ip::address& multicast_address)
    : socket_(io_service)
  {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        listen_address, multicast_port);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    // Join the multicast group.
    socket_.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));

    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        std::bind(&receiver::handle_receive_from, this,
		  std::placeholders::_1,
		  std::placeholders::_2));
  }

  void handle_receive_from(const boost::system::error_code& error,
      size_t bytes_recvd)
  {
    if (!error)
    {
      auto recv_stamp = MakeTimestampNow();
      farm_ng_proto::tractor::v1::Announce announce;
      announce.ParseFromArray(static_cast<const void*>(data_), bytes_recvd);
      *announce.mutable_recv_stamp() = recv_stamp;
      //std::cout << sender_endpoint_ << " : " <<  announce.ShortDebugString() << std::endl;
      announce.set_host( sender_endpoint_.address().to_string());
      announcements_[sender_endpoint_] = announce;
      socket_.async_receive_from(
          boost::asio::buffer(data_, max_length), sender_endpoint_,
          std::bind(&receiver::handle_receive_from, this,
		    std::placeholders::_1,
		    std::placeholders::_2));
    }
  }

  const std::map<boost::asio::ip::udp::endpoint, farm_ng_proto::tractor::v1::Announce>& announcements() const {
    return announcements_;
  }

  void clear_stale_announcements() {
    auto now = google::protobuf::util::TimeUtil::TimestampToSeconds(MakeTimestampNow());
    std::vector< boost::asio::ip::udp::endpoint> keys_to_remove;
    for(const auto& it: announcements_) {
      auto recv_time = google::protobuf::util::TimeUtil::TimestampToSeconds(it.second.recv_stamp());
      if (now - recv_time > 10) {
	keys_to_remove.push_back(it.first);
      }
    }
    for (const auto& k: keys_to_remove) {
      //std::cout << "Removing stale service." << k << std::endl;
      announcements_.erase(k);
    }
  }

private:
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::map< boost::asio::ip::udp::endpoint, farm_ng_proto::tractor::v1::Announce> announcements_;
  enum { max_length = 1024 };
  char data_[max_length];
};



} // namespace
typedef boost::signals2::signal<void(const farm_ng_proto::tractor::v1::Event&)> EventSignal;
typedef std::shared_ptr<EventSignal> EventSignalPtr;


class EventBusImpl
{
public:
  EventBusImpl(boost::asio::io_service& io_service,
      const boost::asio::ip::address& listen_address,
      const boost::asio::ip::address& multicast_address)
    : recv_(io_service, listen_address, multicast_address),
      announce_timer_(io_service),
      socket_(io_service),
      announce_endpoint_(multicast_address, multicast_port),
      signal_(new EventSignal)
  {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        listen_address, 0);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);



    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        std::bind(&EventBusImpl::handle_receive_from, this,
		  std::placeholders::_1,
		  std::placeholders::_2));

    send_announce(boost::system::error_code());
  }

  void send_announce(const boost::system::error_code& error) {
    if(error) {
      std::cerr << "announce timer error: " << error << std::endl;
      return;
    }
    announce_timer_.expires_from_now(boost::posix_time::seconds(1));
    announce_timer_.async_wait(std::bind(&EventBusImpl::send_announce, this, std::placeholders::_1));

    auto endpoint = socket_.local_endpoint();
    farm_ng_proto::tractor::v1::Announce announce;
    announce.set_host(endpoint.address().to_string());
    announce.set_port(endpoint.port());
    announce.set_service("cpp-ipc");

    *announce.mutable_stamp() = MakeTimestampNow();
    announce.SerializeToString(&announce_message_);

    recv_.clear_stale_announcements();

    socket_.send_to(boost::asio::buffer(announce_message_), announce_endpoint_);
    for(const auto& it: recv_.announcements()) {
      boost::asio::ip::udp::endpoint ep(
						     boost::asio::ip::address::from_string(it.second.host()),
						     multicast_port);
      socket_.send_to(boost::asio::buffer(announce_message_), ep);
    }
  }

  void handle_receive_from(const boost::system::error_code& error,
      size_t bytes_recvd)
  {
    if (!error)
    {
      farm_ng_proto::tractor::v1::Event event;
      event.ParseFromArray(static_cast<const void*>(data_), bytes_recvd);

      state_[event.name()] = event;
      (*signal_)(event);

      socket_.async_receive_from(
          boost::asio::buffer(data_, max_length), sender_endpoint_,
          std::bind(&EventBusImpl::handle_receive_from, this,
		      std::placeholders::_1, std::placeholders::_2));
    }
  }
  receiver recv_;
private:


  boost::asio::deadline_timer announce_timer_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint announce_endpoint_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];
  std::string announce_message_;

public:
  std::map<std::string, farm_ng_proto::tractor::v1::Event> state_;
  EventSignalPtr signal_;
};


boost::asio::io_service::id EventBus::id;

void EventBus::shutdown_service() {
}

  EventBus::EventBus(boost::asio::io_service& io_service):
    boost::asio::io_service::service(io_service),
    impl_(new EventBusImpl(io_service,
			   boost::asio::ip::address::from_string("0.0.0.0"),
			   boost::asio::ip::address::from_string(g_multicast_address))) {
  }

  EventBus::~EventBus() {
    impl_.reset(nullptr);
  }

  EventSignalPtr EventBus::GetEventSignal() const {
    return impl_->signal_;
  }

const std::map<std::string, farm_ng_proto::tractor::v1::Event>& EventBus::GetState() const {
    return impl_->state_;
  }
  const std::map<boost::asio::ip::udp::endpoint, farm_ng_proto::tractor::v1::Announce>&
  EventBus::GetAnnouncements() const {
    return impl_->recv_.announcements();
  }


} // namespace farm_ng
