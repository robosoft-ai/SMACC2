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

// LOCAL CLIENT
#include "sm_data_sharing_1/clients/cl_data/cl_data.hpp"
#include "sm_data_sharing_1/clients/cl_data/client_behaviors/cb_store_data1.hpp"
#include "sm_data_sharing_1/clients/cl_data/client_behaviors/cb_store_data2.hpp"
#include "sm_data_sharing_1/clients/cl_data/client_behaviors/cb_process_data.hpp"

// ORTHOGONALS
#include "sm_data_sharing_1/orthogonals/or_data.hpp"
#include "sm_data_sharing_1/orthogonals/or_timer.hpp"
#include "sm_data_sharing_1/orthogonals/or_keyboard.hpp"

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;
using namespace cl_keyboard;

namespace sm_data_sharing_1
{
// FORWARD DECLARATIONS
class StAcquireData1;
class StAcquireData2;
class StProcessData;

//--------------------------------------------------------------------
// STATE MACHINE
struct SmDataSharing1 : public smacc2::SmaccStateMachineBase<SmDataSharing1, StAcquireData1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrData>();
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
  }
};

}  // namespace sm_data_sharing_1

#include "sm_data_sharing_1/states/st_acquire_data1.hpp"
#include "sm_data_sharing_1/states/st_acquire_data2.hpp"
#include "sm_data_sharing_1/states/st_process_data.hpp"
