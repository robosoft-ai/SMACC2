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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_moveit2z
{
  using namespace std::chrono_literals;

class CbSleepFor : public smacc2::SmaccAsyncClientBehavior
{
public:

  CbSleepFor(rclcpp::Duration sleeptime)
    : sleeptime_(sleeptime)
  {
  }

  void onEntry() override
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(sleeptime_.nanoseconds()));
    this->postSuccessEvent();
  }

  void onExit() override
  {
  }

private:
  rclcpp::Duration sleeptime_;
};
}  // namespace sm_husky_barrel_search_1
