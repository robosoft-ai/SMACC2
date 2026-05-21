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
#include <cl_keyboard/cl_keyboard.hpp>

// CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>

// LOCAL CLIENT (no component — data lives on SsMission)
#include "sm_data_sharing_2/clients/cl_data/cl_data.hpp"

// ORTHOGONALS
#include "sm_data_sharing_2/orthogonals/or_data.hpp"
#include "sm_data_sharing_2/orthogonals/or_timer.hpp"
#include "sm_data_sharing_2/orthogonals/or_keyboard.hpp"

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;
using namespace cl_keyboard;

namespace sm_data_sharing_2
{
// Forward declarations
struct SsMission;

//--------------------------------------------------------------------
// STATE MACHINE
// Initial state is SsMission (a superstate), which immediately enters StAcquireData1.
struct SmDataSharing2 : public smacc2::SmaccStateMachineBase<SmDataSharing2, SsMission>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrData>();
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
  }
};

}  // namespace sm_data_sharing_2

// Include order matters:
//   1. ss_mission.hpp  — defines SsMission and Position2D; forward-declares inner states
//   2. Behaviors       — include ss_mission.hpp only (no leaf state dependency)
//   3. States          — include ss_mission.hpp + their behavior header; use context<>()
//
// This ordering resolves the otherwise-circular dependency between behaviors that
// need the superstate type and states that need the behavior type.

#include "sm_data_sharing_2/superstates/ss_mission.hpp"

#include "sm_data_sharing_2/clients/cl_data/client_behaviors/cb_store_data1.hpp"
#include "sm_data_sharing_2/clients/cl_data/client_behaviors/cb_store_data2.hpp"
#include "sm_data_sharing_2/clients/cl_data/client_behaviors/cb_process_data.hpp"

#include "sm_data_sharing_2/states/st_acquire_data1.hpp"
#include "sm_data_sharing_2/states/st_acquire_data2.hpp"
#include "sm_data_sharing_2/states/st_process_data.hpp"
