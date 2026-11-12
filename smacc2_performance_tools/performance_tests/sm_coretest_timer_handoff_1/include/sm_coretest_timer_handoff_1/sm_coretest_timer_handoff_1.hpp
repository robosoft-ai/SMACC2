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

#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>
#include <smacc2/smacc.hpp>
#include <sr_event_countdown/sr_event_countdown.hpp>

#include <sm_coretest_timer_handoff_1/orthogonals/or_sleep.hpp>
#include <sm_coretest_timer_handoff_1/orthogonals/or_timer.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace smacc2::client_behaviors;
using namespace smacc2::state_reactors;
using namespace cl_ros2_timer;
using namespace std::chrono_literals;

namespace sm_coretest_timer_handoff_1
{
struct StA;
struct StB;

struct SmCoretestTimerHandoff1 : public smacc2::SmaccStateMachineBase<SmCoretestTimerHandoff1, StA>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrSleep>();
    this->createOrthogonal<OrTimer>();
  }
};
}  // namespace sm_coretest_timer_handoff_1

#include <sm_coretest_timer_handoff_1/states/st_a.hpp>
#include <sm_coretest_timer_handoff_1/states/st_b.hpp>
