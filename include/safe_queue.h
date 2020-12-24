/**
 * @file safe_queue.h
 * @brief Header file for the ThreadsafeQueue class.
 * @author Yifei Kang
 */

#ifndef SAFE_QUEUE_H_
#define SAFE_QUEUE_H_

#include <unistd.h>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

/**
 * @brief It is a thread-safe FIFO queue.
 *
 */
template <typename QueueType>
class ConcurrentSafeQueue {
 public:
  ConcurrentSafeQueue() { shutdown_ = false; }
  ~ConcurrentSafeQueue() { Shutdown(); }

  /**
   * @brief Shut down the queue, and notify all blocking thread to wake up.
   *
   */
  void Shutdown() {
    shutdown_ = true;
    NotifyAll();
  }

  void NotifyAll() {
    condition_empty_.notify_all();
    condition_full_.notify_all();
  }

  void Push(const QueueType &value) {
    std::lock_guard<std::mutex> guard(mutex_);
    queue_.push(value);
    // Pushing one element, the queue is not empty.
    condition_empty_.notify_all();
  }

  bool Empty() {
    bool empty;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      empty = queue_.empty();
    }
    return empty;

    //     std::lock_guard<std::mutex> guard(mutex_);
    //     return queue_.empty();
  }

  size_t Size() {
    std::lock_guard<std::mutex> guard(mutex_);
    return queue_.size();
  }


  /**
   * @brief get the queue data. If queue is empty, Blocking.
   *
   * @param Output Value.
   * @return True if the value is poped.
   */
  bool Pop(QueueType *value) {
    while (!shutdown_) {
      std::unique_lock<std::mutex> guard(mutex_);
      if (queue_.empty()) {
        condition_empty_.wait(guard);
        continue;
      }
      *value = queue_.front();
      queue_.pop();
      condition_full_.notify_all();
      return true;
    }
    return false;
  }

  /**
   * @brief Push to the queue if the size is less than max_queue_size, else
   * block
   *
   * @param value Input
   * @param max_queue_size Max Size.
   * @return False if shutdown
   */
  bool PushBlockingIfFull(const QueueType &value, size_t max_queue_size) {
    while (!shutdown_) {
      std::unique_lock<std::mutex> guard(mutex_);
      size_t size = queue_.size();
      if (size >= max_queue_size) {
        condition_full_.wait(guard);
        continue;
      }
      queue_.push(value);
      condition_empty_.notify_all();
      return true;
    }
    return false;
  }

  /**
   * @brief Get the value, if the queue is empty wait for given time.
   *
   * @param value Input
   * @param timeout_ns Maximum waiting time
   * @return True if the value is poped.
   */
  bool PopTimeout(QueueType *value, int64_t timeout_ns) {
    std::unique_lock<std::mutex> guard(mutex_);
    if (queue_.empty()) {
      condition_empty_.wait_for(guard, std::chrono::nanoseconds(timeout_ns));
    }
    if (queue_.empty()) {
      return false;
    }
    *value = queue_.front();
    queue_.pop();
    condition_full_.notify_all();
    return true;
  }

  /**
   * @brief Get a copy of the front element.
   * @param Output value
   * @return True if getting the copy.
   */
  bool GetCopyFromFront(QueueType *value) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (queue_.empty()) {
      return false;
    }
    *value = queue_.front();
    return true;
  }
  /**
   * @brief Get a copy of the back element.
   * @param Output value
   * @return True if getting the copy.
   */
  bool GetCopyFromBack(QueueType *value) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (queue_.empty()) {
      return false;
    }
    *value = queue_.back();
    return true;
  }
  std::atomic_bool shutdown_;  // Flag if shutdown

 private:
  std::mutex mutex_;             // The queue mutex
  std::queue<QueueType> queue_;  // Actual queue.
  std::condition_variable condition_empty_;
  std::condition_variable condition_full_;
};

#endif  // SAFE_QUEUE_H_
