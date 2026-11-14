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

#include <string>

#include <example_interfaces/action/fibonacci.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>
#include <smacc2/smacc.hpp>

namespace sm_coretest_action_client_1
{
// Sends one Fibonacci goal through the action client component named "server".
class CbFibonacci : public smacc2::client_behavior_bases::CbActionClientBehaviorBase<
                      example_interfaces::action::Fibonacci>
{
public:
  explicit CbFibonacci(int order, const std::string & actionClientName = "server") : order_(order)
  {
    this->setActionClientName(actionClientName);
  }

  void onEntry() override
  {
    Goal goal;
    goal.order = order_;
    RCLCPP_INFO(getLogger(), "[CbFibonacci] sending a goal of order %d", order_);
    this->sendGoal(goal);
  }

private:
  int order_;
};

// The same goal through the component aimed at an action nothing serves.
class CbFibonacciNoServer : public CbFibonacci
{
public:
  CbFibonacciNoServer() : CbFibonacci(5, "no_server") {}
};
}  // namespace sm_coretest_action_client_1
