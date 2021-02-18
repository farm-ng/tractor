#ifndef FARM_NG_EVENT_LOG_H_
#define FARM_NG_EVENT_LOG_H_
#include <memory>
#include <string>

#include <boost/filesystem.hpp>

#include "farm_ng/core/io.pb.h"
#include "farm_ng/core/resource.pb.h"

namespace farm_ng {
namespace core {

class EventLogWriterImpl;
class EventLogWriter {
 public:
  EventLogWriter(const boost::filesystem::path& log_path);
  ~EventLogWriter();
  void Write(const Event& event);
  void WriteAsResource(const Event& event);
 private:
  std::unique_ptr<EventLogWriterImpl> impl_;
};

}  // namespace core
}  // namespace farm_ng

#endif
