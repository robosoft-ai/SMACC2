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

#include <sm_husky_barrel_search_1/clients/led_array/cl_led_array.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace sm_husky_barrel_search_1
{
namespace cl_led_array
{
using namespace std::chrono_literals;

class CbSequenceColorBlinking : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override
  {
    cl_led_array::ClLedArray* ledarray;
    this->requiresClient(ledarray);

    while (!isShutdownRequested())
    {
      ledarray->turnOn(LedColor::RED);
      rclcpp::sleep_for(1s);
      ledarray->turnOff(LedColor::RED);
      ledarray->turnOn(LedColor::YELLOW);
      rclcpp::sleep_for(1s);
      ledarray->turnOff(LedColor::YELLOW);
      ledarray->turnOn(LedColor::GREEN);
      rclcpp::sleep_for(1s);
      ledarray->turnOff(LedColor::GREEN);
    }
  }

  void onExit() override
  {
  }
};
}  // namespace cl_led_array
}  // namespace sm_husky_barrel_search_1
