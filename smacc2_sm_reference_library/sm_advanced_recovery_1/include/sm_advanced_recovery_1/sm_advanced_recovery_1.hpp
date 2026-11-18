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

#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_ros2_timer/cl_ros2_timer.hpp>

// ORTHOGONALS
#include <sm_advanced_recovery_1/orthogonals/or_keyboard.hpp>
#include <sm_advanced_recovery_1/orthogonals/or_subscriber.hpp>
#include <sm_advanced_recovery_1/orthogonals/or_timer.hpp>

using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace sm_advanced_recovery_1::cl_subscriber;

//CLIENT BEHAVIORS
#include <sm_advanced_recovery_1/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.hpp>
#include <sm_advanced_recovery_1/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.hpp>

#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <cl_ros2_timer/client_behaviors/cb_ros_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_advanced_recovery_1
{
//SUPERSTATES
struct SsACycle;
namespace a_cycle_inner_states
{
struct StiACycleLoop;
struct StiACycleStep1;
struct StiACycleStep2;
struct StiACycleStep3;
struct StiACycleStep4;
struct StiACycleStep5;
struct StiACycleStep6;
struct StiACycleStep7;
struct StiACycleStep8;
struct StiACycleStep9;
}  // namespace a_cycle_inner_states

struct SsBCycle;

namespace b_cyclenner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiBCycleLoop;
struct StiBCycleStep1;
struct StiBCycleStep2;
struct StiBCycleStep3;
struct StiBCycleStep4;
struct StiBCycleStep5;
struct StiBCycleStep6;
struct StiBCycleStep7;
struct StiBCycleStep8;
struct StiBCycleStep9;
}  // namespace b_cyclenner_states

struct SsCCycle;
namespace c_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiCCycleLoop;
struct StiCCycleStep1;
struct StiCCycleStep2;
struct StiCCycleStep3;
struct StiCCycleStep4;
struct StiCCycleStep5;
struct StiCCycleStep6;
struct StiCCycleStep7;
struct StiCCycleStep8;
struct StiCCycleStep9;
}  // namespace c_cycle_inner_states

//STATES
struct StObserve;
struct StRecoverStep1;
struct StRecoverStep2;
struct StRecoverStep3;
struct StRecoverStep4;
struct StRecoverStep5;
struct StRecoverStep6;
struct StRecoverStep7;

//MODE STATES
class MsRun;
class MsRecover;

struct EvToDeep : sc::event<EvToDeep>{};
struct EvFail : sc::event<EvFail>{};

// STATE MACHINE
struct SmAdvancedRecovery1 : public smacc2::SmaccStateMachineBase<SmAdvancedRecovery1, MsRun>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrSubscriber>();
  }
};
}  // namespace sm_advanced_recovery_1

// MODE STATES
#include <sm_advanced_recovery_1/mode_states/ms_recover.hpp>
#include <sm_advanced_recovery_1/mode_states/ms_run.hpp>

//STATES
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_1.hpp>
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_2.hpp>
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_3.hpp>
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_4.hpp>
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_5.hpp>
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_6.hpp>
#include <sm_advanced_recovery_1/states/ms_recover_inner_states/st_recover_step_7.hpp>

#include <sm_advanced_recovery_1/states/st_observe.hpp>

#include <sm_advanced_recovery_1/superstates/ss_a_cycle.hpp>
#include <sm_advanced_recovery_1/superstates/ss_b_cycle.hpp>
#include <sm_advanced_recovery_1/superstates/ss_c_cycle.hpp>

//ss_ac_cycle
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_4.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_3.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_1.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_loop.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_2.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_5.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_6.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_7.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_8.hpp>
#include <sm_advanced_recovery_1/states/a_cycle_inner_states/sti_a_cycle_step_9.hpp>

//ss_b_cycle
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_4.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_3.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_1.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_loop.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_2.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_5.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_6.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_7.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_8.hpp>
#include <sm_advanced_recovery_1/states/b_cycle_inner_states/sti_b_cycle_step_9.hpp>
//ss_c_cycle
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_4.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_3.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_1.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_loop.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_2.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_5.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_6.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_7.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_8.hpp>
#include <sm_advanced_recovery_1/states/c_cycle_inner_states/sti_c_cycle_step_9.hpp>
