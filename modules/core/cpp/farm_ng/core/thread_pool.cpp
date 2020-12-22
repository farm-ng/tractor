#include "farm_ng/core/thread_pool.h"

#include <glog/logging.h>

namespace farm_ng {
namespace core {

ThreadPool::ThreadPool() {}

void ThreadPool::Stop() { io_service_.stop(); }

void ThreadPool::Start(size_t n_threads) {
  CHECK_GT(n_threads, 0);
  CHECK(threads_.empty()) << "ThreadPool already started. Call Join().";
  io_service_.reset();
  for (size_t i = 0; i < n_threads; ++i) {
    threads_.emplace_back([this]() { io_service_.run(); });
  }
}

void ThreadPool::Join() {
  for (auto& thread : threads_) {
    thread.join();
  }
  threads_.clear();
}

boost::asio::io_service& ThreadPool::get_io_service() { return io_service_; }

}  // namespace core
}  // namespace farm_ng
