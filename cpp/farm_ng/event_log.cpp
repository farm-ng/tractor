#include "farm_ng/event_log.h"

#include <stdexcept>
#include <fstream>


namespace farm_ng {
  class EventLogWriterImpl {
  public:
    EventLogWriterImpl(std::string log_path) : log_path_(log_path), out_(log_path_, std::ofstream::binary) {
    }
    void Write(const farm_ng_proto::tractor::v1::Event& event) {
      std::string packet;
      event.SerializeToString(&packet);
      if(packet.size() > std::numeric_limits<uint16_t>::max()) {
	throw std::invalid_argument("Event is too large");
      }
      uint16_t n_bytes = packet.size();
      out_.write(reinterpret_cast<const char*>(&n_bytes),
		 sizeof(n_bytes));
      out_ << packet;
      out_.flush();
    }

    std::string log_path_;
    std::ofstream out_;
  };
  EventLogWriter::EventLogWriter(std::string log_path):
      impl_(new EventLogWriterImpl(log_path)) {
  }
  EventLogWriter::~EventLogWriter()  {
    impl_.reset(nullptr);
  }

    void EventLogWriter::Write(const farm_ng_proto::tractor::v1::Event& event) {
      impl_->Write(event);
    }

} // namespace farm_ng
