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

// Core test: the features no other core test reaches.
//
//   StGenerator      an event generator posts EvFalse on six updates, then EvTrue
//   StSequence       a CbSequence of three sleeps runs to its end: EvCbSuccess
//   StSequenceAbort  the second step of a CbSequence posts its failure: EvCbFailure
//   StPeriodic       an updatable behavior with a 200 ms update period, for 1.1 s
//   StSequenceCut    a CbSequence of ten sleeps is cut short by a 330 ms timer
//   StSequenceCutOnStep  the same, cut at 200 ms, just as the sequence starts its second step
//
// The states follow each other in that order and start again. Each sends an outcome it must not see
// to StTrap, which is terminal.

#pragma once

#include <smacc2/smacc.hpp>

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <eg_conditional_generator/eg_conditional_generator.hpp>
#include <smacc2/client_behaviors/cb_sequence.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>

#include <sm_coretest_features_1/client_behaviors/cb_fail.hpp>
#include <sm_coretest_features_1/client_behaviors/cb_periodic_tick.hpp>
#include <sm_coretest_features_1/orthogonals/or_timer.hpp>
#include <sm_coretest_features_1/orthogonals/or_work.hpp>

using namespace boost;
using namespace smacc2;
using namespace smacc2::default_events;
using namespace smacc2::client_behaviors;
using namespace smacc2::event_generators;
using namespace cl_ros2_timer;
using namespace std::chrono_literals;

namespace sm_coretest_features_1
{
struct StGenerator;
struct StSequence;
struct StSequenceAbort;
struct StPeriodic;
struct StSequenceCut;
struct StSequenceCutOnStep;
struct StTrap;

struct SmCoretestFeatures1 : public smacc2::SmaccStateMachineBase<SmCoretestFeatures1, StGenerator>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrWork>();
  }
};
}  // namespace sm_coretest_features_1

#include <sm_coretest_features_1/states/st_generator.hpp>
#include <sm_coretest_features_1/states/st_periodic.hpp>
#include <sm_coretest_features_1/states/st_sequence.hpp>
#include <sm_coretest_features_1/states/st_sequence_abort.hpp>
#include <sm_coretest_features_1/states/st_sequence_cut.hpp>
#include <sm_coretest_features_1/states/st_sequence_cut_on_step.hpp>
#include <sm_coretest_features_1/states/st_trap.hpp>
