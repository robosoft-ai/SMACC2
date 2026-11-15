// Copyright 2026 RobosoftAI Inc.
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

#include <cstdlib>
#include <vector>

#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_coretest_idle_load_1
{
// Counts the messages the subscriber delivers through its signal, and is an updatable.
class CbMessageCounter : public smacc2::SmaccClientBehavior, public smacc2::ISmaccUpdatable
{
public:
  void onEntry() override
  {
    const char * leak = std::getenv("SM_CORETEST_IDLE_LEAK");
    leak_ = leak != nullptr && leak[0] == '1';
    this->requiresComponent(subscriber_);
    subscriber_->onMessageReceived(&CbMessageCounter::onMessage, this);
  }

  void update() override { ++updates_; }

private:
  void onMessage(const std_msgs::msg::String & msg)
  {
    if (leak_) kept_.push_back(msg);  // the deliberate leak, for checking the soak itself
    if (++messages_ % 1000 == 0)
    {
      // FATAL so that the count is printed with logging otherwise off
      RCLCPP_FATAL(getLogger(), "[idle] messages: %lu updates: %lu", messages_, updates_);
    }
  }

  smacc2::client_core_components::CpTopicSubscriber<std_msgs::msg::String> * subscriber_ = nullptr;
  std::vector<std_msgs::msg::String> kept_;
  bool leak_ = false;
  unsigned long messages_ = 0;
  unsigned long updates_ = 0;
};
}  // namespace sm_coretest_idle_load_1
