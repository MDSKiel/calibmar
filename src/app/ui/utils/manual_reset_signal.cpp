#include "manual_reset_signal.h"

ManualResetSignal::ManualResetSignal(bool initial) : flag_(initial) {}

void ManualResetSignal::Set() {
  std::lock_guard<std::mutex> lock(mutex_);
  flag_ = true;
  signal_.notify_one();
}

void ManualResetSignal::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  flag_ = false;
}

bool ManualResetSignal::Wait() {
  std::unique_lock<std::mutex> lock(mutex_);
  signal_.wait(lock, [this]() { return flag_; });
  return true;
}
