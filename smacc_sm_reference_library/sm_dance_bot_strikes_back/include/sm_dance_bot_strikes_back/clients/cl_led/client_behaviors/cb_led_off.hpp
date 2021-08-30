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

#pragma once

#include <sm_dance_bot_strikes_back/clients/cl_led/cl_led.hpp>
#include <smacc2/smacc.hpp>

namespace sm_dance_bot_strikes_back
{
namespace cl_led
{
class CbLEDOff : public smacc2::SmaccClientBehavior
{
public:
  cl_led::ClLED * ledActionClient_;

  void onEntry() override
  {
    this->requiresClient(ledActionClient_);

    cl_led::ClLED::Goal goal;
    goal.command = cl_led::ClLED::Goal::CMD_OFF;
    ledActionClient_->sendGoal(goal);
  }

  void onExit() override
  {
    // RCLCPP_INFO(getLogger(), "Entering ToolClientBehavior");
  }
};
}  // namespace cl_led
}  // namespace sm_dance_bot_strikes_back
