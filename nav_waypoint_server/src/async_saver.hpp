// Copyright (c) 2026 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ASYNC_SAVER_HPP_
#define ASYNC_SAVER_HPP_

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace nav_waypoint_server
{
// Bounded queue; filesystem work never runs in the ROS callback executor.
class AsyncSaver
{
public:
  AsyncSaver()
  : worker_([this]() {run();}) {}
  ~AsyncSaver()
  {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_ = true;}
    ready_.notify_one();
    worker_.join();
  }
  bool submit(std::function<void()> job)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || jobs_.size() >= 4) {return false;}
    jobs_.push_back(std::move(job));
    ready_.notify_one();
    return true;
  }

private:
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::function<void()>> jobs_;
  bool stopping_{false};
  std::thread worker_;
  void run()
  {
    for (;; ) {
      std::function<void()> job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this]() {return stopping_ || !jobs_.empty();});
        if (jobs_.empty()) {return;}
        job = std::move(jobs_.front());
        jobs_.pop_front();
      }
      job();
    }
  }
};
}  // namespace nav_waypoint_server
#endif  // ASYNC_SAVER_HPP_
