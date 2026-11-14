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

// Core test: the action client component and the action behavior base, with no simulator.
//
// OrAction holds an action server on the state machine's own node and two action client components.
// Five states follow each other, one per outcome of a goal:
//
//   StSuccess    the goal succeeds                                    EvCbSuccess -> StRejected
//   StRejected   the server rejects the goal                          EvCbFailure -> StAborted
//   StAborted    the server aborts the goal                           EvCbFailure -> StNoServer
//   StNoServer   the action has no server                             EvCbFailure -> StCancelled
//   StCancelled  the state exits on a timer with the goal in flight   EvTimer     -> StSuccess
//
// Each state sends the outcome it must not see to StTrap, which is terminal. StSuccess follows
// StCancelled on purpose: the cancelled goal's result arrives while StSuccess's own goal runs, and
// must not be taken for it.

#pragma once

#include <smacc2/smacc.hpp>

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

#include <sm_coretest_action_client_1/client_behaviors/cb_fibonacci.hpp>
#include <sm_coretest_action_client_1/orthogonals/or_action.hpp>
#include <sm_coretest_action_client_1/orthogonals/or_timer.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace cl_ros2_timer;
using namespace std::chrono_literals;

namespace sm_coretest_action_client_1
{
struct StSuccess;
struct StRejected;
struct StAborted;
struct StNoServer;
struct StCancelled;
struct StTrap;

struct SmCoretestActionClient1
: public smacc2::SmaccStateMachineBase<SmCoretestActionClient1, StSuccess>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrAction>();
  }
};
}  // namespace sm_coretest_action_client_1

#include <sm_coretest_action_client_1/states/st_aborted.hpp>
#include <sm_coretest_action_client_1/states/st_cancelled.hpp>
#include <sm_coretest_action_client_1/states/st_no_server.hpp>
#include <sm_coretest_action_client_1/states/st_rejected.hpp>
#include <sm_coretest_action_client_1/states/st_success.hpp>
#include <sm_coretest_action_client_1/states/st_trap.hpp>
