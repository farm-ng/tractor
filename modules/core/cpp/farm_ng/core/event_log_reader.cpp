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
    uint16_t n_bytes_u16;
    in_.read(reinterpret_cast<char*>(&n_bytes_u16), sizeof(n_bytes_u16));

    if (!in_) {
      throw std::runtime_error("Could not read packet length header");
    }
    uint64_t n_bytes;
    if(n_bytes_u16 == 0) { // magic tell if we're larger than uint16 bytes.
      in_.read(reinterpret_cast<char*>(&n_bytes), sizeof(n_bytes));
      if (!in_) {
        throw std::runtime_error("Could not read packet length header");
      }
    } else {
      n_bytes = n_bytes_u16;
    }
    std::string ss;
    ss.resize(n_bytes);
    in_.read(ss.data(), n_bytes);
    if (!in_) {
      throw std::runtime_error("Could not read event data.");
    }
    farm_ng::core::Event event;
    event.ParseFromString(ss);

    core::Resource resource;
    if (event.data().UnpackTo(&resource)) {
      if (resource.content_type() == ContentTypeProtobufBinary<Event>()) {
        event = ReadProtobufFromResource<core::Event>(resource);
      }
    }
    return event;
  }
  std::string log_path_;
  std::ifstream in_;
};

EventLogReader::EventLogReader(std::string log_path)
    : impl_(new EventLogReaderImpl(log_path)) {}

EventLogReader::EventLogReader(farm_ng::core::Resource resource)
    : impl_(new EventLogReaderImpl(
          NativePathFromResourcePath(resource).string())) {}

EventLogReader::~EventLogReader() { impl_.reset(nullptr); }

void EventLogReader::Reset(std::string log_path) {
  impl_.reset(new EventLogReaderImpl(log_path));
}

farm_ng::core::Event EventLogReader::ReadNext() { return impl_->ReadNext(); }

}  // namespace core
}  // namespace farm_ng
