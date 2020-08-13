#include <iostream>
#include <stdexcept>
#include "farm_ng/event_log_reader.h"

int main() {
  farm_ng::EventLogReader reader("/tmp/farm-ng-event.log");
  int count = 0;
  while (true) {
    try {
      std::cout << reader.ReadNext().ShortDebugString() << std::endl;
      count++;
    } catch (std::exception& e) {
      std::cerr << "Exception: " << e.what() << "\n";
      break;
    }
  }
  std::cerr << "n messages: " << count << std::endl;
}
