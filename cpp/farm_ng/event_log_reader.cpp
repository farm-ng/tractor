#include "farm_ng/event_log_reader.h"

#include <stdexcept>
#include <fstream>


namespace farm_ng {
  class EventLogReaderImpl {
  public:
    EventLogReaderImpl(std::string log_path) : log_path_(log_path), in_(log_path_, std::ofstream::binary) {
    }

    farm_ng_proto::tractor::v1::Event ReadNext() {
      uint16_t n_bytes;
      in_.read(reinterpret_cast<char*>(&n_bytes), sizeof(n_bytes));
      if(!in_) {
	throw std::runtime_error("Could not read packet length header");
      }
      std::string ss; ss.resize(n_bytes);
      in_.read(ss.data(), n_bytes);
      if(!in_) {
	throw std::runtime_error("Could not read event data.");
      }
      farm_ng_proto::tractor::v1::Event event;
      event.ParseFromString(ss);
      return event;
    }
    std::string log_path_;
    std::ifstream in_;
  };
  EventLogReader::EventLogReader(std::string log_path):
      impl_(new EventLogReaderImpl(log_path)) {
  }
  EventLogReader::~EventLogReader()  {
    impl_.reset(nullptr);
  }

    farm_ng_proto::tractor::v1::Event EventLogReader::ReadNext() {
      return impl_->ReadNext();
    }

} // namespace farm_ng
