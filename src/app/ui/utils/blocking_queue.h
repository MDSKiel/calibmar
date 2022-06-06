#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>

namespace calibmar {

  template <typename T>
  class BlockingQueue {
   private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<T> queue_;

   public:
    void push(T const& value) {
      {
        std::unique_lock<std::mutex> lock(this->mutex_);
        queue_.push_front(value);
      }
      this->condition_.notify_one();
    }

    void push(T&& value) {
      {
        std::unique_lock<std::mutex> lock(this->mutex_);
        queue_.push_front(std::move(value));
      }
      this->condition_.notify_one();
    }

    T pop_wait() {
      std::unique_lock<std::mutex> lock(this->mutex_);
      this->condition_.wait(lock, [=] { return !this->queue_.empty(); });
      T value(std::move(this->queue_.back()));
      this->queue_.pop_back();
      return value;
    }

    template <class _Rep, class _Period>
    bool pop_wait_for(const std::chrono::duration<_Rep, _Period>& wait_for, T* value) {
      std::unique_lock<std::mutex> lock(this->mutex_);
      if (this->condition_.wait_for(lock, wait_for, [=] { return !this->queue_.empty(); })) {
        *value = std::move(this->queue_.back());
        this->queue_.pop_back();
        return true;
      }

      return false;
    }
  };
}
