#ifndef FARM_NG_EVENT_LOG_READER_H_
#define FARM_NG_EVENT_LOG_READER_H_

#include <memory>
#include <string>

#include "farm_ng/core/io.pb.h"
#include "farm_ng/core/resource.pb.h"

namespace farm_ng {
namespace core {

typedef farm_ng::core::Event EventPb;

class EventLogReaderImpl;
class EventLogReader {
 public:
  explicit EventLogReader(std::string log_path);
  explicit EventLogReader(Resource log_path);
  ~EventLogReader();

  void Reset(std::string log_path);

  EventPb ReadNext();

 private:
  std::unique_ptr<EventLogReaderImpl> impl_;
};

}  // namespace core
}  // namespace farm_ng

#endif
