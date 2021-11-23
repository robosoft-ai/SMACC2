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

//
// Author: Denis Štogl (template)
//

#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

namespace sm_coretest_transition_speed_1
{
//STATES
struct State1;
struct State2;

//VARIABLES - shared between states (using "_<name>_"-syntax to make this obvious)
static unsigned int _counter_ = 1;
std::shared_ptr<rclcpp::Node> _node_;
rclcpp::Time _start_time_;

unsigned int _sum_of_iterations_ = 0.0;
double _sum_of_elapsed_time_ = 0.0;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmCoretestTransitionSpeed1
: public smacc2::SmaccStateMachineBase<SmCoretestTransitionSpeed1, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    _node_ = std::make_shared<rclcpp::Node>("sm_coretest_transition_speed_1");
    _start_time_ = _node_->now();
  }
};

}  // namespace sm_coretest_transition_speed_1

#include "states/st_state_1.hpp"
#include "states/st_state_2.hpp"
