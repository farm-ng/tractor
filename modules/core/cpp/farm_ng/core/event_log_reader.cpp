#include "farm_ng/core/event_log_reader.h"
#include "farm_ng/core/blobstore.h"

#include <fstream>
#include <stdexcept>

namespace farm_ng {
namespace core {

class EventLogReaderImpl {
 public:
  EventLogReaderImpl(std::string log_path)
      : log_path_(log_path), in_(log_path_, std::ofstream::binary) {
    if (!in_) {
      throw std::runtime_error("Could not open file");
    }
  }

  farm_ng::core::Event ReadNext() {
    uint16_t n_bytes;
    in_.read(reinterpret_cast<char*>(&n_bytes), sizeof(n_bytes));
    if (!in_) {
      throw std::runtime_error("Could not read packet length header");
    }
    std::string ss;
    ss.resize(n_bytes);
    in_.read(ss.data(), n_bytes);
    if (!in_) {
      throw std::runtime_error("Could not read event data.");
    }
    farm_ng::core::Event event;
    event.ParseFromString(ss);
    return event;
  }
  std::string log_path_;
  std::ifstream in_;
};

EventLogReader::EventLogReader(std::string log_path)
    : impl_(new EventLogReaderImpl(log_path)) {}

EventLogReader::EventLogReader(farm_ng::core::Resource resource)
    : impl_(new EventLogReaderImpl(
          (GetBlobstoreRoot() / resource.path()).string())) {}

EventLogReader::~EventLogReader() { impl_.reset(nullptr); }

void EventLogReader::Reset(std::string log_path) {
  impl_.reset(new EventLogReaderImpl(log_path));
}

farm_ng::core::Event EventLogReader::ReadNext() { return impl_->ReadNext(); }

}  // namespace core
}  // namespace farm_ng
