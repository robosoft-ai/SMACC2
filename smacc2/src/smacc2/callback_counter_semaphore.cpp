// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 *****************************************************************************************************************/
#include <boost/signals2.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/callback_counter_semaphore.hpp>
#include <thread>

namespace smacc2
{
CallbackCounterSemaphore::CallbackCounterSemaphore(std::string name, int count)
: count_(count), name_(name)
{
}

bool CallbackCounterSemaphore::acquire()
{
  std::unique_lock<std::mutex> lock(mutex_);
  RCLCPP_DEBUG(
    rclcpp::get_logger(name_), "[CallbackCounterSemaphore] acquire callback %s %ld", name_.c_str(),
    (long)this);

  if (finalized)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger(name_), "[CallbackCounterSemaphore] callback rejected %s %ld",
      name_.c_str(), (long)this);
    return false;
  }

  ++count_;
  cv_.notify_one();

  RCLCPP_DEBUG(
    rclcpp::get_logger(name_), "[CallbackCounterSemaphore] callback accepted %s %ld", name_.c_str(),
    (long)this);
  return true;
}

void CallbackCounterSemaphore::release()
{
  std::unique_lock<std::mutex> lock(mutex_);
  --count_;
  cv_.notify_one();

  RCLCPP_DEBUG(
    rclcpp::get_logger(name_), "[CallbackCounterSemaphore] callback finished %s %ld", name_.c_str(),
    (long)this);
}

void CallbackCounterSemaphore::finalize()
{
  std::unique_lock<std::mutex> lock(mutex_);

  while (count_ > 0)
  {
    cv_.wait(lock);
  }
  finalized = true;

  for (auto conn : connections_)
  {
    conn.disconnect();
  }

  connections_.clear();
  RCLCPP_DEBUG(
    rclcpp::get_logger(name_), "[CallbackCounterSemaphore] callbacks finalized %s %ld",
    name_.c_str(), (long)this);
}

void CallbackCounterSemaphore::addConnection(boost::signals2::connection conn)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (finalized)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger(name_),
      "[CallbackCounterSemaphore] ignoring adding callback, already finalized %s %ld",
      name_.c_str(), (long)this);
    return;
  }

  connections_.push_back(conn);
}

}  // namespace smacc2
