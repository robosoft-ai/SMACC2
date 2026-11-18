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

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>

//CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

// ORTHOGONALS
#include "orthogonals/or_timer.hpp"

using namespace boost;
using namespace smacc2;

namespace sm_branching
{
//STATE
struct State1;
struct State2;
struct State2b;
struct State2c;
struct State3;
struct State3b;
struct State3c;
struct State4;
struct State4c;
struct State5;
struct State5b;
struct State6;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmBranching : public smacc2::SmaccStateMachineBase<SmBranching, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override { this->createOrthogonal<OrTimer>(); }
};

}  // namespace sm_branching

#include "states/st_state_1.hpp"
#include "states/st_state_2.hpp"
#include "states/st_state_2b.hpp"
#include "states/st_state_2c.hpp"
#include "states/st_state_3.hpp"
#include "states/st_state_3b.hpp"
#include "states/st_state_3c.hpp"
#include "states/st_state_4.hpp"
#include "states/st_state_4c.hpp"
#include "states/st_state_5.hpp"
#include "states/st_state_5b.hpp"
#include "states/st_state_6.hpp"
