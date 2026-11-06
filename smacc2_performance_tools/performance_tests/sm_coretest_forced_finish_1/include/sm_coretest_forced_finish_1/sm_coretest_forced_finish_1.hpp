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

// Core test: forced finish of an asynchronous client behavior.
//
// StA runs CbSleepFor(30s) and a 1 s timer. The timer always wins, so on every
// StA exit the state machine thread calls requestForceFinish() on the sleep
// behavior while its onEntry thread polls isShutdownRequested(). The behavior
// then posts EvCbSuccess and EvCbFinished with CURRENT_STATE lifetime while the
// machine is STATE_EXITING, so both must hit the discard path in
// ISmaccStateMachine::postEvent.
//
// StRest reacts to either of those events by entering StTrap. StRest has no
// CbSleepFor, so any such event seen there crossed the StA exit: it was not
// discarded because the posting thread read a stale stateMachineCurrentAction.
// StTrap is terminal and must never be entered.

#pragma once

#include <smacc2/smacc.hpp>

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>
#include <sr_event_countdown/sr_event_countdown.hpp>

#include <sm_coretest_forced_finish_1/orthogonals/or_sleep.hpp>
#include <sm_coretest_forced_finish_1/orthogonals/or_timer.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;
using namespace smacc2::client_behaviors;
using namespace cl_ros2_timer;
using namespace std::chrono_literals;

namespace sm_coretest_forced_finish_1
{
struct StA;
struct StRest;
struct StTrap;

struct SmCoretestForcedFinish1
: public smacc2::SmaccStateMachineBase<SmCoretestForcedFinish1, StA>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrSleep>();
  }
};
}  // namespace sm_coretest_forced_finish_1

#include <sm_coretest_forced_finish_1/states/st_a.hpp>
#include <sm_coretest_forced_finish_1/states/st_rest.hpp>
#include <sm_coretest_forced_finish_1/states/st_trap.hpp>
