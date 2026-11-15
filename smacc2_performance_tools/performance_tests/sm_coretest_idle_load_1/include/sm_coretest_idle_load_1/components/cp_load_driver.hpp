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

#include <string>

#include <smacc2/client_core_components/cp_ros2_timer.hpp>
#include <smacc2/client_core_components/cp_topic_publisher.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_coretest_idle_load_1
{
// Publishes a 1 kB message on every tick of the client's timer, and is an updatable.
class CpLoadDriver : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  void onInitialize() override
  {
    this->requiresComponent(timer_);
    this->requiresComponent(publisher_);
    message_.data.assign(1024, 'x');
    timer_->onTimerTick(&CpLoadDriver::onTick, this);
  }

  void update() override { ++updates_; }

private:
  void onTick() { publisher_->publish(message_); }

  smacc2::client_core_components::CpRos2Timer * timer_ = nullptr;
  smacc2::client_core_components::CpTopicPublisher<std_msgs::msg::String> * publisher_ = nullptr;
  std_msgs::msg::String message_;
  unsigned long updates_ = 0;
};
}  // namespace sm_coretest_idle_load_1
