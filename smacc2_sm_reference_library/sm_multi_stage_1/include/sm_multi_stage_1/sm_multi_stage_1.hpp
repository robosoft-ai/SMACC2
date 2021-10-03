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
class SsACCycle;
namespace ac_cycle_inner_states
{
class StiACCycleLoop;
class StiACCycleInspire;
class StiACCyclePlateau;
class StiACCycleExpire;
class StiACCycleDwell;
class StiACCyclePulse;
class StiACCycleTitrate;
class StiACCycleRinse;
class StiACCycleRecycle;
class StiACCyclePush;
}  // namespace ac_cycle_inner_states

class SsCMVCycle;

namespace cmv_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCMVCycleLoop;
class StiCMVCycleInspire;
class StiCMVCyclePlateau;
class StiCMVCycleExpire;
class StiCMVCycleDwell;
class StiCMVCyclePulse;
class StiCMVCycleTitrate;
class StiCMVCycleRinse;
class StiCMVCycleRecycle;
class StiCMVCyclePush;
}  // namespace cmv_cycle_inner_states


//STATES
class StObserve;
class StLeakyLungStep1;
class StLeakyLungStep2;
class StLeakyLungStep3;
class StLeakyLungStep4;
class StLeakyLungStep5;
class StLeakyLungStep6;
class StLeakyLungStep7;

//MODE STATES
class MsRun1;
class MsLeakyLung1;

struct EvToDeep : sc::event<EvToDeep>{};
struct EvFail : sc::event<EvFail>{};

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
#include <sm_multi_stage_1/mode_states/ms_leaky_lung_1.hpp>
#include <sm_multi_stage_1/mode_states/ms_run_1.hpp>

//STATES
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_1.hpp>
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_2.hpp>
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_3.hpp>
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_4.hpp>
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_5.hpp>
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_6.hpp>
#include <sm_multi_stage_1/states/ms_leaky_lung_1_inner_states/st_leaky_lung_step_7.hpp>

#include <sm_multi_stage_1/states/st_observe.hpp>

#include <sm_multi_stage_1/superstates/ss_ac_cycle.hpp>
#include <sm_multi_stage_1/superstates/ss_cmv_cycle.hpp>

//ss_ac_cycle
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_dwell.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_expire.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_inspire.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_loop.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_plateau.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_pulse.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_titrate.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_rinse.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_recycle.hpp>
#include <sm_multi_stage_1/states/ac_cycle_inner_states/sti_ac_cycle_push.hpp>

//ss_cmv_cycle
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_dwell.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_expire.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_inspire.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_loop.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_plateau.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_pulse.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_titrate.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_rinse.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_recycle.hpp>
#include <sm_multi_stage_1/states/cmv_cycle_inner_states/sti_cmv_cycle_push.hpp>

