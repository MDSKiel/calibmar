#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

class AutoResetSignal {
 public:
  explicit AutoResetSignal(bool initial = false);
  AutoResetSignal(const AutoResetSignal&) = delete;
  AutoResetSignal& operator=(const AutoResetSignal&) = delete;  // non-copyable

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
bool AutoResetSignal::WaitFor(const std::chrono::duration<Rep, Period>& wait_for) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (signal_.wait_for(lock, wait_for, [this]() { return flag_; })) {
    flag_ = false;  // auto reset
    return true;
  }

  return false;
}