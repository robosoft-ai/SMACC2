// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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

// ORTHOGONALS
#include "sm_hercules_1/orthogonals/or_timer.hpp"

namespace sm_hercules_1
{
// SMACC2 clases
using sm_hercules_1::OrTimer;

//STATES
struct State1;
struct State2;

//VARIABLES - shared between states (using "_<name>_"-syntax to make this obvious)
std::shared_ptr<rclcpp::Node> _node_;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmHercules1
: public smacc2::SmaccStateMachineBase<SmHercules1, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    _node_ = std::make_shared<rclcpp::Node>("sm_hercules_1");
  }
};

}  // namespace sm_hercules_1

// STATES
#include "states/st_state_1.hpp"
#include "states/st_state_2.hpp"
