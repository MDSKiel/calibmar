#include "auto_reset_signal.h"

AutoResetSignal::AutoResetSignal(bool initial) : flag_(initial) {}

void AutoResetSignal::Set() {
  std::lock_guard<std::mutex> lock(mutex_);
  flag_ = true;
  signal_.notify_one();
}

void AutoResetSignal::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  flag_ = false;
}

bool AutoResetSignal::Wait() {
  std::unique_lock<std::mutex> lock(mutex_);
  signal_.wait(lock, [this]() { return flag_; });
  flag_ = false;  // auto reset
  return true;
}
