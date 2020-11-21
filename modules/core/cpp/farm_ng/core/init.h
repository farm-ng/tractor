#ifndef FARM_NG_INIT_H_
#define FARM_NG_INIT_H_
#include <boost/asio/io_service.hpp>

#include "farm_ng/core/ipc.h"

namespace farm_ng {
namespace core {

void GlogFailureFunction();

int Main(int argc, char** argv, int (*main_func)(EventBus&),
         void (*cleanup_func)(EventBus&));

}  // namespace core
}  // namespace farm_ng

#endif
