#ifndef FARM_NG_IPC_H_
#define FARM_NG_IPC_H_
#include <memory>
#include <functional>
#include <map>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "farm_ng_proto/tractor/v1/io.pb.h"

namespace farm_ng {

typedef boost::signals2::signal<void(const farm_ng_proto::tractor::v1::Event&)> EventSignal;
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
    const std::map<std::string, farm_ng_proto::tractor::v1::Event>& GetState() const;

    const std::map<boost::asio::ip::udp::endpoint, farm_ng_proto::tractor::v1::Announce>&
      GetAnnouncements() const;

  private:
    std::unique_ptr<EventBusImpl> impl_;
};

inline EventBus& GetEventBus(boost::asio::io_service& io_service) {
   return boost::asio::use_service<EventBus>(io_service);
}

}

#endif
