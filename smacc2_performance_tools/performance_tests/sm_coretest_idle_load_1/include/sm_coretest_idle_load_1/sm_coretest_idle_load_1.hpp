// Copyright 2026 RobosoftAI Inc.
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

// Core test for memory: a machine that stays in one state while the polling loop works.
//
//   StIdle is entered once and never left. While it is active
//     - a 500 Hz timer fires, and CpLoadDriver publishes a 1 kB message on each tick
//     - a CpTopicSubscriber receives the messages, posts EvTopicMessage (which no state consumes)
//       and emits its signal to CbMessageCounter
//     - CpLoadDriver and CbMessageCounter are updatables, called on every loop iteration
//
// Run by smacc2_performance_tools/core_test_tools/memory_tests/idle_load_soak.sh, which samples the
// process's memory, threads and file descriptors. Every 1000 messages received it logs a count at
// FATAL. With SM_CORETEST_IDLE_LEAK=1 in the environment CbMessageCounter keeps every message: a
// deliberate leak that the soak must fail on.

#pragma once

#include <smacc2/smacc.hpp>

#include <sm_coretest_idle_load_1/client_behaviors/cb_message_counter.hpp>
#include <sm_coretest_idle_load_1/orthogonals/or_load.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;

namespace sm_coretest_idle_load_1
{
struct StIdle;

struct SmCoretestIdleLoad1 : public smacc2::SmaccStateMachineBase<SmCoretestIdleLoad1, StIdle>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override { this->createOrthogonal<OrLoad>(); }
};
}  // namespace sm_coretest_idle_load_1

#include <sm_coretest_idle_load_1/states/st_idle.hpp>
