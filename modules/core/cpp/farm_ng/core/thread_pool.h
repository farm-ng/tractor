#include <thread>

#include <boost/asio/io_service.hpp>

namespace farm_ng {
namespace core {

class ThreadPool {
 public:
  ThreadPool();

  void Stop();

  void Start(size_t n_threads);

  void Join();

  boost::asio::io_service& get_io_service();

 private:
  boost::asio::io_service io_service_;
  std::vector<std::thread> threads_;
};

}  // namespace core
}  // namespace farm_ng
