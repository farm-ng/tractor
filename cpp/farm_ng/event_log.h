#ifndef FARM_NG_EVENT_LOG_H_
#define FARM_NG_EVENT_LOG_H_
#include <memory>
#include <string>

#include <boost/filesystem.hpp>
#include "farm_ng_proto/tractor/v1/io.pb.h"
#include "farm_ng_proto/tractor/v1/resource.pb.h"
namespace farm_ng {
class EventLogWriterImpl;
class EventLogWriter {
 public:
  EventLogWriter(const boost::filesystem::path& log_path);
  ~EventLogWriter();
  void Write(const farm_ng_proto::tractor::v1::Event& event);

 private:
  std::unique_ptr<EventLogWriterImpl> impl_;
};

}  // namespace farm_ng

#endif
