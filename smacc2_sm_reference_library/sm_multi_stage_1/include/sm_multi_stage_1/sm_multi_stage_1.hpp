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

class SsACCycle3;
namespace ac_cycle_3
{
class StiACCycleLoop3;
class StiACCycleInspire3;
class StiACCyclePlateau3;
class StiACCycleExpire3;
class StiACCycleDwell3;
class StiACCyclePulse3;
class StiACCycleTitrate3;
class StiACCycleRinse3;
class StiACCycleRecycle3;
class StiACCyclePush3;
}  // namespace ac_cycle_3

class SsACCycle4;
namespace ac_cycle_4
{
class StiACCycleLoop4;
class StiACCycleInspire4;
class StiACCyclePlateau4;
class StiACCycleExpire4;
class StiACCycleDwell4;
class StiACCyclePulse4;
class StiACCycleTitrate4;
class StiACCycleRinse4;
class StiACCycleRecycle4;
class StiACCyclePush4;
}  // namespace ac_cycle_4

class SsACCycle5;
namespace ac_cycle_5
{
class StiACCycleLoop5;
class StiACCycleInspire5;
class StiACCyclePlateau5;
class StiACCycleExpire5;
class StiACCycleDwell5;
class StiACCyclePulse5;
class StiACCycleTitrate5;
class StiACCycleRinse5;
class StiACCycleRecycle5;
class StiACCyclePush5;
}  // namespace ac_cycle_5


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

class SsCMVCycle3;

namespace cmv_cycle_3
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCMVCycleLoop3;
class StiCMVCycleInspire3;
class StiCMVCyclePlateau3;
class StiCMVCycleExpire3;
class StiCMVCycleDwell3;
class StiCMVCyclePulse3;
class StiCMVCycleTitrate3;
class StiCMVCycleRinse3;
class StiCMVCycleRecycle3;
class StiCMVCyclePush3;
}  // namespace cmv_cycle_3

class SsCMVCycle4;

namespace cmv_cycle_4
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCMVCycleLoop4;
class StiCMVCycleInspire4;
class StiCMVCyclePlateau4;
class StiCMVCycleExpire4;
class StiCMVCycleDwell4;
class StiCMVCyclePulse4;
class StiCMVCycleTitrate4;
class StiCMVCycleRinse4;
class StiCMVCycleRecycle4;
class StiCMVCyclePush4;
}  // namespace cmv_cycle_4

class SsCMVCycle5;

namespace cmv_cycle_5
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCMVCycleLoop5;
class StiCMVCycleInspire5;
class StiCMVCyclePlateau5;
class StiCMVCycleExpire5;
class StiCMVCycleDwell5;
class StiCMVCyclePulse5;
class StiCMVCycleTitrate5;
class StiCMVCycleRinse5;
class StiCMVCycleRecycle5;
class StiCMVCyclePush5;
}  // namespace cmv_cycle_5

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

class StObserve3;
class StObserve4;
class StObserve5;


class ACCycleLoop1;
class CMVCycleLoop1;
class ACCycleLoop2;
class CMVCycleLoop2;
class ACCycleLoop3;
class CMVCycleLoop3;
class ACCycleLoop4;
class CMVCycleLoop4;
class ACCycleLoop5;
class CMVCycleLoop5;


//MODE STATES
class MsRun1;
class MsRecovery1;

//MODE STATES
class MsRun2;
class MsRecovery2;

class MsRun3;
class MsRun4;
class MsRun5;

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
#include <sm_multi_stage_1/mode_states/ms_run_3.hpp>
#include <sm_multi_stage_1/mode_states/ms_run_4.hpp>
#include <sm_multi_stage_1/mode_states/ms_run_5.hpp>

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

#include <sm_multi_stage_1/states/ac_cycle_loop_1.hpp>
#include <sm_multi_stage_1/states/ac_cycle_loop_2.hpp>
#include <sm_multi_stage_1/states/ac_cycle_loop_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_loop_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_loop_5.hpp>

#include <sm_multi_stage_1/states/cmv_cycle_loop_1.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_loop_2.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_loop_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_loop_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_loop_5.hpp>

#include <sm_multi_stage_1/states/st_observe_1.hpp>
#include <sm_multi_stage_1/states/st_observe_2.hpp>
#include <sm_multi_stage_1/states/st_observe_3.hpp>
#include <sm_multi_stage_1/states/st_observe_4.hpp>
#include <sm_multi_stage_1/states/st_observe_5.hpp>

#include <sm_multi_stage_1/superstates/ss_ac_cycle_1.hpp>
#include <sm_multi_stage_1/superstates/ss_ac_cycle_2.hpp>
#include <sm_multi_stage_1/superstates/ss_ac_cycle_3.hpp>
#include <sm_multi_stage_1/superstates/ss_ac_cycle_4.hpp>
#include <sm_multi_stage_1/superstates/ss_ac_cycle_5.hpp>


#include <sm_multi_stage_1/superstates/ss_cmv_cycle_1.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle_2.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle_3.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle_4.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle_5.hpp>


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

//ss_ac_cycle_3
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_dwell_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_expire_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_inspire_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_loop_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_plateau_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_pulse_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_push_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_recycle_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_rinse_3.hpp>
#include <sm_multi_stage_1/states/ac_cycle_3/sti_ac_cycle_titrate_3.hpp>

//ss_ac_cycle_4
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_dwell_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_expire_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_inspire_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_loop_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_plateau_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_pulse_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_push_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_recycle_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_rinse_4.hpp>
#include <sm_multi_stage_1/states/ac_cycle_4/sti_ac_cycle_titrate_4.hpp>

//ss_ac_cycle_5
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_dwell_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_expire_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_inspire_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_loop_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_plateau_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_pulse_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_push_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_recycle_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_rinse_5.hpp>
#include <sm_multi_stage_1/states/ac_cycle_5/sti_ac_cycle_titrate_5.hpp>

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

//ss_cmv_cycle_2
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

//ss_cmv_cycle_3
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_dwell_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_expire_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_inspire_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_loop_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_plateau_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_pulse_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_push_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_recycle_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_rinse_3.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_3/sti_cmv_cycle_titrate_3.hpp>

//ss_cmv_cycle_4
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_dwell_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_expire_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_inspire_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_loop_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_plateau_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_pulse_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_push_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_recycle_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_rinse_4.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_4/sti_cmv_cycle_titrate_4.hpp>

//ss_cmv_cycle_5
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_dwell_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_expire_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_inspire_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_loop_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_plateau_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_pulse_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_push_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_recycle_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_rinse_5.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_5/sti_cmv_cycle_titrate_5.hpp>
