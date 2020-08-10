#ifndef FARM_NG_EVENT_LOG_READER_H_
#define FARM_NG_EVENT_LOG_READER_H_
#include <memory>
#include <string>

#include "farm_ng_proto/tractor/v1/io.pb.h"
namespace farm_ng {


  class EventLogReaderImpl;
  class EventLogReader {
  public:
    EventLogReader(std::string log_path);
    ~EventLogReader();

    farm_ng_proto::tractor::v1::Event ReadNext();

  private:
    std::unique_ptr<EventLogReaderImpl> impl_;
  };

} // namespace farm_ng

#endif
