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

// Core test: user code that throws.
//
// StA, SsPlain (holding StRest), SsEscalate (holding StInner) and StRetry follow each other on
// 500 ms timers.
// SM_CORETEST_THROW_SITE names the one site that throws in a run; with none, nothing throws. Each
// throw is announced with a "THROWING at <site>" line, so a run shows how many throws the machine
// took and whether it kept cycling.
//
// StA holds the sites the core logs and continues from: its onExit, a synchronous behavior, an
// asynchronous behavior and an updatable behavior, and the orthogonal's client is updatable too.
// The other sites are failed state initializations, which post EvStInitializationFailure:
//   state_entry      StA, top level, no reaction: the state machine's hook runs, the machine stays in StA
//   rest_entry       StRest inside SsPlain, neither reacts: escalates through SsPlain to the hook
//   inner_entry      StInner, no reaction of its own: escalates to SsEscalate, which goes to StSafe
//   inner_configure  the same from StInner's runtimeConfigure
//   container_entry  SsEscalate itself: StInner is not initialized, SsEscalate's reaction goes to StSafe
//   retry_entry      StRetry reacts to its own failure by entering itself again

#pragma once

#include <smacc2/smacc.hpp>

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

#include <sm_coretest_exceptions_1/client_behaviors/cb_async_throw.hpp>
#include <sm_coretest_exceptions_1/client_behaviors/cb_sync_throw.hpp>
#include <sm_coretest_exceptions_1/client_behaviors/cb_update_throw.hpp>
#include <sm_coretest_exceptions_1/orthogonals/or_throw.hpp>
#include <sm_coretest_exceptions_1/orthogonals/or_timer.hpp>
#include <sm_coretest_exceptions_1/throw_site.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace cl_ros2_timer;
using namespace std::chrono_literals;

namespace sm_coretest_exceptions_1
{
struct StA;
struct StRest;
struct SsEscalate;
struct SsPlain;
struct StInner;
struct StRetry;
struct StSafe;

struct SmCoretestExceptions1 : public smacc2::SmaccStateMachineBase<SmCoretestExceptions1, StA>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrThrow>();
  }

  void onUnhandledStInitializationFailure(
    const std::string & stateName, const std::string & what) override
  {
    RCLCPP_WARN(getLogger(), "HOOK onUnhandledStInitializationFailure: %s", stateName.c_str());
    ISmaccStateMachine::onUnhandledStInitializationFailure(stateName, what);
  }
};
}  // namespace sm_coretest_exceptions_1

#include <sm_coretest_exceptions_1/superstates/ss_escalate.hpp>
#include <sm_coretest_exceptions_1/superstates/ss_plain.hpp>

#include <sm_coretest_exceptions_1/states/st_a.hpp>
#include <sm_coretest_exceptions_1/states/st_inner.hpp>
#include <sm_coretest_exceptions_1/states/st_rest.hpp>
#include <sm_coretest_exceptions_1/states/st_retry.hpp>
#include <sm_coretest_exceptions_1/states/st_safe.hpp>
