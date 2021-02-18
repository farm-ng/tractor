#include "farm_ng/core/event_log.h"
#include "farm_ng/core/blobstore.h"
#include "farm_ng/core/ipc.h"

#include <fstream>
#include <stdexcept>

namespace farm_ng {
namespace core {

class EventLogWriterImpl {
 public:
  EventLogWriterImpl(const boost::filesystem::path& log_path)
      : log_path_(log_path), out_(log_path_.string(), std::ofstream::binary) {}
  void Write(const farm_ng::core::Event& event) {
    std::string packet;
    event.SerializeToString(&packet);
    if (packet.size() > std::numeric_limits<uint16_t>::max()) {
      uint16_t magic = 0;
      out_.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
      uint64_t n_bytes = packet.size();
      out_.write(reinterpret_cast<const char*>(&n_bytes), sizeof(n_bytes));
    } else {
      uint16_t n_bytes = packet.size();
      out_.write(reinterpret_cast<const char*>(&n_bytes), sizeof(n_bytes));
    }
    out_ << packet;
    out_.flush();
  }

  boost::filesystem::path log_path_;
  std::ofstream out_;
};
EventLogWriter::EventLogWriter(const boost::filesystem::path& log_path)
    : impl_(new EventLogWriterImpl(log_path)) {}

EventLogWriter::~EventLogWriter() { impl_.reset(nullptr); }

void EventLogWriter::Write(const Event& event) {
  impl_->Write(event);
}
void EventLogWriter::WriteAsResource(const Event& event) {
  auto resource_path = GetUniqueArchiveResource(
      event.name(), "pb", ContentTypeProtobufBinary<Event>());
  WriteProtobufToBinaryFile(resource_path.second, event);
  Write(MakeEvent(event.name(), resource_path.first, event.stamp()));
}

}  // namespace core
}  // namespace farm_ng
