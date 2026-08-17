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

#include <cl_keyboard/cl_keyboard.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_core_components/cp_topic_publisher.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_nav2_gazebo_test_3
{

class OrKeyboard : public smacc2::Orthogonal<OrKeyboard>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_keyboard::ClKeyboard>();

    // arrow-key teleop output, consumed by the behavior server's
    // assisted_teleop collision guard (CbKeyboardTwistTeleop publishes here)
    client->createComponent<
      smacc2::client_core_components::CpTopicPublisher<geometry_msgs::msg::Twist>>(
      "/cmd_vel_teleop");
  }
};

}  // namespace sm_nav2_gazebo_test_3
