#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

class ManualResetSignal {
 public:
  explicit ManualResetSignal(bool initial = false);
  ManualResetSignal(const ManualResetSignal&) = delete;
  ManualResetSignal& operator=(const ManualResetSignal&) = delete;  // non-copyable

  void Set();
  void Reset();

  bool Wait();
  template <class Rep, class Period>
  bool WaitFor(const std::chrono::duration<Rep, Period>& wait_for);

 private:
  bool flag_;
  std::mutex mutex_;
  std::condition_variable signal_;
};

template <class Rep, class Period>
bool ManualResetSignal::WaitFor(const std::chrono::duration<Rep, Period>& wait_for) {
  std::unique_lock<std::mutex> lock(mutex_);
  return (signal_.wait_for(lock, wait_for, [this]() { return flag_; }));
}