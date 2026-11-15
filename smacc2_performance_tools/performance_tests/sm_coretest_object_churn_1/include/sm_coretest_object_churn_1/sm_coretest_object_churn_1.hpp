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

// Core test for memory: every state creates and destroys one of each state-scoped object.
//
//   StChurnA <-> StChurnB, as fast as the polling loop allows. Each state holds
//     - CbTickSubscriber   a synchronous, updatable behavior with a signal connection to a 500 Hz timer
//     - CbFinish           an asynchronous behavior whose thread posts EvCbSuccess
//     - an EgConditionalGenerator that posts EvTrue on its first update
//     - an SrAllEventsGo that waits for both events and posts the event the state leaves on
//     - its own signal connection to the same timer
//
// Run by smacc2_performance_tools/core_test_tools/memory_tests/object_churn_soak.sh, which samples
// the process's memory, threads and file descriptors. Every 1000 round trips it logs a count at FATAL.

#pragma once

#include <smacc2/smacc.hpp>

#include <eg_conditional_generator/eg_conditional_generator.hpp>
#include <sr_all_events_go/sr_all_events_go.hpp>

#include <sm_coretest_object_churn_1/client_behaviors/cb_finish.hpp>
#include <sm_coretest_object_churn_1/client_behaviors/cb_tick_subscriber.hpp>
#include <sm_coretest_object_churn_1/orthogonals/or_async.hpp>
#include <sm_coretest_object_churn_1/orthogonals/or_tick.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace smacc2::event_generators;
using namespace smacc2::state_reactors;

namespace sm_coretest_object_churn_1
{
struct StChurnA;
struct StChurnB;

struct SmCoretestObjectChurn1
: public smacc2::SmaccStateMachineBase<SmCoretestObjectChurn1, StChurnA>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTick>();
    this->createOrthogonal<OrAsync>();
  }

  unsigned long roundTrips_ = 0;
};
}  // namespace sm_coretest_object_churn_1

#include <sm_coretest_object_churn_1/states/st_churn_a.hpp>
#include <sm_coretest_object_churn_1/states/st_churn_b.hpp>
