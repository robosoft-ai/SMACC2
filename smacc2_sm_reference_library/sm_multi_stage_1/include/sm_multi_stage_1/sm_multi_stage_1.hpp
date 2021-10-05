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
#include <sm_multi_stage_1/orthogonals/or_keyboard.hpp>
#include <sm_multi_stage_1/orthogonals/or_subscriber.hpp>
#include <sm_multi_stage_1/orthogonals/or_timer.hpp>
#include <sm_multi_stage_1/orthogonals/or_updatable_publisher.hpp>

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
using namespace cl_keyboard;
using namespace sm_multi_stage_1::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

#include <sm_multi_stage_1/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.hpp>
#include <sm_multi_stage_1/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.hpp>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_multi_stage_1
{
//SUPERSTATES
class SsACCycle1;
namespace ac_cycle_1
{
class StiACCycleLoop1;
class StiACCycleInspire1;
class StiACCyclePlateau1;
class StiACCycleExpire1;
class StiACCycleDwell1;
class StiACCyclePulse1;
class StiACCycleTitrate1;
class StiACCycleRinse1;
class StiACCycleRecycle1;
class StiACCyclePush1;
}  // namespace ac_cycle_1

class SsACCycle2;
namespace ac_cycle_2
{
class StiACCycleLoop2;
class StiACCycleInspire2;
class StiACCyclePlateau2;
class StiACCycleExpire2;
class StiACCycleDwell2;
class StiACCyclePulse2;
class StiACCycleTitrate2;
class StiACCycleRinse2;
class StiACCycleRecycle2;
class StiACCyclePush2;
}  // namespace ac_cycle_2

class SsCMVCycle1;

namespace cmv_cycle_1
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCMVCycleLoop1;
class StiCMVCycleInspire1;
class StiCMVCyclePlateau1;
class StiCMVCycleExpire1;
class StiCMVCycleDwell1;
class StiCMVCyclePulse1;
class StiCMVCycleTitrate1;
class StiCMVCycleRinse1;
class StiCMVCycleRecycle1;
class StiCMVCyclePush1;
}  // namespace cmv_cycle_1

class SsCMVCycle2;

namespace cmv_cycle_2
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCMVCycleLoop2;
class StiCMVCycleInspire2;
class StiCMVCyclePlateau2;
class StiCMVCycleExpire2;
class StiCMVCycleDwell2;
class StiCMVCyclePulse2;
class StiCMVCycleTitrate2;
class StiCMVCycleRinse2;
class StiCMVCycleRecycle2;
class StiCMVCyclePush2;
}  // namespace cmv_cycle_2

//STATES
class StObserve1;
class StRecoveryAnalyze1;
class StRecoveryBifurcate1;
class StRecoveryCalculate1;
class StRecoveryDeliberate1;
class StRecoveryEvaluate1;
class StRecoveryGenerate1;
class StRecoveryInnervate1;

class StObserve2;
class StRecoveryAnalyze2;
class StRecoveryBifurcate2;
class StRecoveryCalculate2;
class StRecoveryDeliberate2;
class StRecoveryEvaluate2;
class StRecoveryGenerate2;
class StRecoveryInnervate2;

//MODE STATES
class MsRun1;
class MsRecovery1;

//MODE STATES
class MsRun2;
class MsRecovery2;

struct EvToDeep : sc::event<EvToDeep>
{
};
struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmMultiStage1 : public smacc2::SmaccStateMachineBase<SmMultiStage1, MsRun1>
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
}  // namespace sm_multi_stage_1

// MODE STATES
#include <sm_multi_stage_1/mode_states/ms_recovery_1.hpp>
#include <sm_multi_stage_1/mode_states/ms_recovery_2.hpp>
#include <sm_multi_stage_1/mode_states/ms_run_1.hpp>
#include <sm_multi_stage_1/mode_states/ms_run_2.hpp>

//STATES
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_analyze_1.hpp>
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_bifurcate_1.hpp>
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_calculate_1.hpp>
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_deliberate_1.hpp>
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_evaluate_1.hpp>
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_generate_1.hpp>
#include <sm_multi_stage_1/states/ms_recovery_1/st_recovery_innervate_1.hpp>

#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_analyze_2.hpp>
#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_bifurcate_2.hpp>
#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_calculate_2.hpp>
#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_deliberate_2.hpp>
#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_evaluate_2.hpp>
#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_generate_2.hpp>
#include <sm_multi_stage_1/states/ms_recovery_2/st_recovery_innervate_2.hpp>

#include <sm_multi_stage_1/states/st_observe_1.hpp>
#include <sm_multi_stage_1/states/st_observe_2.hpp>

#include <sm_multi_stage_1/superstates/ss_ac_cycle_1.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle_1.hpp>

#include <sm_multi_stage_1/superstates/ss_ac_cycle_2.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle_2.hpp>

//ss_ac_cycle_1
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_dwell_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_expire_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_inspire_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_loop_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_plateau_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_pulse_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_push_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_recycle_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_rinse_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_1/sti_ac_cycle_titrate_1.hpp>

//ss_ac_cycle_2
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_dwell_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_expire_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_inspire_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_loop_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_plateau_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_pulse_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_push_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_recycle_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_rinse_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_2/sti_ac_cycle_titrate_2.hpp>

//ss_cmv_cycle_1
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_dwell_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_expire_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_inspire_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_loop_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_plateau_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_pulse_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_push_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_recycle_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_rinse_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_1/sti_cmv_cycle_titrate_1.hpp>

//ss_cmv_cycle_1
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_dwell_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_expire_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_inspire_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_loop_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_plateau_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_pulse_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_push_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_recycle_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_rinse_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_2/sti_cmv_cycle_titrate_2.hpp>