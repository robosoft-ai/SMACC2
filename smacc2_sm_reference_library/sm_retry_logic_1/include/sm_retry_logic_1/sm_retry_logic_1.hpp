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

// sm_retry_logic_1: Demonstrates a state reactor (SrEventCountdown) defined in a
// superstate (SsAttemptTask). The reactor counts timer timeouts and explicit failure
// keypresses across inner StIdle ↔ StAttempting transitions. After 3 failures the
// machine enters StExhausted. Pressing 's' at any time enters StSuccess.
//
// Key interactions:
//   'n' — start an attempt (StIdle → StAttempting)
//   'f' — manually fail the current attempt (StAttempting → StIdle, counts as failure)
//   timer expires (10 s) — auto-fail (StAttempting → StIdle, counts as failure)
//   's' — succeed (→ StSuccess, from StIdle or StAttempting)
//   3 failures total — StExhausted (regardless of which inner state is active)

#pragma once

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_ros2_timer/cl_ros2_timer.hpp>

// CLIENT BEHAVIORS
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

// STATE REACTORS
#include <sr_event_countdown/sr_event_countdown.hpp>

// ORTHOGONALS
#include <sm_retry_logic_1/orthogonals/or_keyboard.hpp>
#include <sm_retry_logic_1/orthogonals/or_timer.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace std::chrono_literals;

namespace sm_retry_logic_1
{
// Forward declarations
struct SsAttemptTask;
struct StIdle;
struct StAttempting;
struct StSuccess;
struct StExhausted;

// STATE MACHINE
struct SmRetryLogic1 : public smacc2::SmaccStateMachineBase<SmRetryLogic1, SsAttemptTask>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
  }
};
}  // namespace sm_retry_logic_1

// Include order: superstate first (forward-declares inner states), then inner states,
// then outer terminal states.
#include <sm_retry_logic_1/superstates/ss_attempt_task.hpp>
#include <sm_retry_logic_1/states/st_idle.hpp>
#include <sm_retry_logic_1/states/st_attempting.hpp>
#include <sm_retry_logic_1/states/st_success.hpp>
#include <sm_retry_logic_1/states/st_exhausted.hpp>
