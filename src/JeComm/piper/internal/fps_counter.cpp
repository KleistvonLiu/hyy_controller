#include "fps_counter.hpp"

#include <chrono>

namespace piper {
namespace gripper {
namespace internal {

FpsCounter::FpsCounter() : running_(false), period_ms_(100) {}

FpsCounter::~FpsCounter() { Stop(); }

void FpsCounter::AddVariable(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  counters_.emplace(name, 0);
  prev_counters_.emplace(name, 0);
  fps_results_.emplace(name, 0.0);
}

void FpsCounter::Increment(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = counters_.find(name);
  if (it != counters_.end()) {
    ++(it->second);
  }
}

double FpsCounter::GetFps(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = fps_results_.find(name);
  if (it == fps_results_.end()) {
    return 0.0;
  }
  return it->second;
}

void FpsCounter::Start(int period_ms) {
  Stop();
  period_ms_ = period_ms;
  running_.store(true);
  worker_ = std::thread(&FpsCounter::WorkerLoop, this);
}

void FpsCounter::Stop() {
  running_.store(false);
  if (worker_.joinable()) {
    worker_.join();
  }
}

void FpsCounter::WorkerLoop() {
  while (running_.load()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double scale = period_ms_ > 0 ? 1000.0 / static_cast<double>(period_ms_) : 0.0;
      for (auto& kv : counters_) {
        const uint64_t current = kv.second;
        const uint64_t previous = prev_counters_[kv.first];
        fps_results_[kv.first] = static_cast<double>(current - previous) * scale;
        prev_counters_[kv.first] = current;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
  }
}

}  // namespace internal
}  // namespace gripper
}  // namespace piper
