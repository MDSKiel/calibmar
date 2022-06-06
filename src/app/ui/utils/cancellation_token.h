#pragma once

#include <atomic>

namespace calibmar {
  class CancellationToken {
   public:
    explicit CancellationToken(std::atomic<bool>& flag);
    CancellationToken(const CancellationToken&) = delete;
    CancellationToken& operator=(const CancellationToken&) = delete;  // non-copyable

    explicit operator bool() const;

   private:
    std::atomic<bool>& flag_;
  };

  inline CancellationToken::CancellationToken(std::atomic<bool>& flag) : flag_(flag) {}

  inline CancellationToken::operator bool() const {
    return flag_.load();
  }
}