#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace piper {
namespace gripper {
namespace internal {

class FpsCounter {
 public:
  FpsCounter();
  ~FpsCounter();

  FpsCounter(const FpsCounter&) = delete;
  FpsCounter& operator=(const FpsCounter&) = delete;

  void AddVariable(const std::string& name);
  void Increment(const std::string& name);
  double GetFps(const std::string& name) const;

  void Start(int period_ms);
  void Stop();

 private:
  void WorkerLoop();

  mutable std::mutex mutex_;
  std::unordered_map<std::string, uint64_t> counters_;
  std::unordered_map<std::string, uint64_t> prev_counters_;
  std::unordered_map<std::string, double> fps_results_;

  std::atomic<bool> running_;
  int period_ms_;
  std::thread worker_;
};

}  // namespace internal
}  // namespace gripper
}  // namespace piper
