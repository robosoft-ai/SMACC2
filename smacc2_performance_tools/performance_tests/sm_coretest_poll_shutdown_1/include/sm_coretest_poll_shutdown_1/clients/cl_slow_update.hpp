// Copyright 2025 Robosoft Inc.
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
#pragma once

#include <chrono>
#include <thread>

#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>

namespace sm_coretest_poll_shutdown_1
{
// An updatable client whose update() blocks for slow_update_ms. SignalDetector::pollOnce calls
// it on the polling thread, inside the window between the loop's rclcpp::ok() check and its
// Rate::sleep(). With slow_update_ms close to the polling period, an rclcpp shutdown lands in
// that window in most runs. The sleep must stay shorter than the period: rclcpp::Rate::sleep()
// does not sleep at all when the iteration overran, and then never reaches the throwing call.
class ClSlowUpdate : public smacc2::ISmaccClient, public smacc2::ISmaccUpdatable
{
public:
  void update() override
  {
    auto node = getNode();
    if (!node->has_parameter("slow_update_ms"))
    {
      node->declare_parameter<int>("slow_update_ms", 150);
      node->get_parameter("slow_update_ms", slowUpdateMs_);
      RCLCPP_INFO(getLogger(), "[ClSlowUpdate] update() blocks for %d ms", slowUpdateMs_);
    }
    updates_++;
    RCLCPP_INFO_THROTTLE(
      getLogger(), *node->get_clock(), 1000, "[ClSlowUpdate] update #%d", updates_);
    std::this_thread::sleep_for(std::chrono::milliseconds(slowUpdateMs_));
  }

private:
  int slowUpdateMs_ = 150;
  int updates_ = 0;
};
}  // namespace sm_coretest_poll_shutdown_1
