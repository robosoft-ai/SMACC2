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
// StA and StRest alternate on 500 ms timers. StA holds every site: its own onEntry and onExit, a
// synchronous behavior, an asynchronous behavior and an updatable behavior, and the orthogonal's
// client is updatable too. SM_CORETEST_THROW_SITE names the one site that throws in a run; with
// none, nothing throws. Each throw is announced with a "THROWING at <site>" line, so a run shows
// how many throws the machine took and whether it kept alternating.

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

struct SmCoretestExceptions1 : public smacc2::SmaccStateMachineBase<SmCoretestExceptions1, StA>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrThrow>();
  }
};
}  // namespace sm_coretest_exceptions_1

#include <sm_coretest_exceptions_1/states/st_a.hpp>
#include <sm_coretest_exceptions_1/states/st_rest.hpp>
