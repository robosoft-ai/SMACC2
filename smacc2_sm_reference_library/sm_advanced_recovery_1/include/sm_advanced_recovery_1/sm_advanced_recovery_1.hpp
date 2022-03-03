// Copyright 2021 RobosoftAI Inc.
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
#include <keyboard_client/cl_keyboard.hpp>
#include <ros_timer_client/cl_ros_timer.hpp>

// ORTHOGONALS
#include <sm_advanced_recovery_1/orthogonals/or_keyboard.hpp>
#include <sm_advanced_recovery_1/orthogonals/or_subscriber.hpp>
#include <sm_advanced_recovery_1/orthogonals/or_timer.hpp>
#include <sm_advanced_recovery_1/orthogonals/or_updatable_publisher.hpp>

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
using namespace cl_keyboard;
using namespace sm_advanced_recovery_1::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

#include <sm_advanced_recovery_1/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.hpp>
#include <sm_advanced_recovery_1/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.hpp>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_advanced_recovery_1
{
//SUPERSTATES
class SsACycle;
namespace a_cycle_inner_states
{
class StiACycleLoop;
class StiACycleStep1;
class StiACycleStep2;
class StiACycleStep3;
class StiACycleStep4;
class StiACycleStep5;
class StiACycleStep6;
class StiACycleStep7;
class StiACycleStep8;
class StiACycleStep9;
}  // namespace a_cycle_inner_states

class SsBCycle;

namespace b_cyclenner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBCycleLoop;
class StiBCycleStep1;
class StiBCycleStep2;
class StiBCycleStep3;
class StiBCycleStep4;
class StiBCycleStep5;
class StiBCycleStep6;
class StiBCycleStep7;
class StiBCycleStep8;
class StiBCycleStep9;
}  // namespace b_cyclenner_states

class SsCCycle;
namespace c_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCCycleLoop;
class StiCCycleStep1;
class StiCCycleStep2;
class StiCCycleStep3;
class StiCCycleStep4;
class StiCCycleStep5;
class StiCCycleStep6;
class StiCCycleStep7;
class StiCCycleStep8;
class StiCCycleStep9;
}  // namespace c_cycle_inner_states

//STATES
class StObserve;
class StRecoverStep1;
class StRecoverStep2;
class StRecoverStep3;
class StRecoverStep4;
class StRecoverStep5;
class StRecoverStep6;
class StRecoverStep7;

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
    this->createOrthogonal<OrUpdatablePublisher>();
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
