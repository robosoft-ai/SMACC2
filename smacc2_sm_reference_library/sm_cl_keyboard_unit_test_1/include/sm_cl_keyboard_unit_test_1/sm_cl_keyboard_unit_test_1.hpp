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

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_ros2_timer/components/cp_timer_listener_1.hpp>

#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_keyboard/components/cp_keyboard_listener_1.hpp>

//CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>

// ORTHOGONALS
#include "orthogonals/or_timer.hpp"
#include "orthogonals/or_keyboard.hpp"

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace cl_keyboard::components;

namespace sm_cl_keyboard_unit_test_1
{
//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmClKeyboardUnitTest1 : public smacc2::SmaccStateMachineBase<SmClKeyboardUnitTest1, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override { 
    this->createOrthogonal<OrTimer>(); 
    this->createOrthogonal<OrKeyboard>(); 
  }
};

}  // namespace sm_cl_keyboard_unit_test_1

#include "states/st_state_1.hpp"
#include "states/st_state_2.hpp"
