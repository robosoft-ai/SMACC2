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
 ******************************************************************************************************************/

#pragma once
#include <boost/signals2.hpp>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace smacc2
{
class CallbackCounterSemaphore
{
public:
  CallbackCounterSemaphore(std::string name, int count = 0);
  bool acquire();

  void release();

  void finalize();

  void addConnection(boost::signals2::connection conn);

private:
  int count_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<boost::signals2::connection> connections_;
  bool finalized = false;
  std::string name_;
};
}  // namespace smacc2
