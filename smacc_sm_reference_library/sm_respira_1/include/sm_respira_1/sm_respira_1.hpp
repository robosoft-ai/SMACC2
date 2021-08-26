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

#include <smacc/smacc.h>
#include <rclcpp/rclcpp.hpp>

// CLIENTS
#include <keyboard_client/cl_keyboard.h>
#include <ros_timer_client/cl_ros_timer.h>

// ORTHOGONALS
#include <sm_respira_1/orthogonals/or_keyboard.h>
#include <sm_respira_1/orthogonals/or_subscriber.h>
#include <sm_respira_1/orthogonals/or_timer.h>
#include <sm_respira_1/orthogonals/or_updatable_publisher.h>

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
using namespace cl_keyboard;
using namespace sm_respira_1::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>

#include <sm_respira_1/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.h>
#include <sm_respira_1/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.h>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;

namespace sm_respira_1
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
}  // namespace cmv_cycle_inner_states

class SsPCCycle;
namespace pc_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiPCCycleLoop;
class StiPCCycleInspire;
class StiPCCyclePlateau;
class StiPCCycleExpire;
class StiPCCycleDwell;
}  // namespace pc_cycle_inner_states

class SsPSCycle;
namespace ps_cycle_inner_states
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiPSCycleLoop;
class StiPSCycleInspire;
class StiPSCyclePlateau;
class StiPSCycleExpire;
class StiPSCycleDwell;
}  // namespace ps_cycle_inner_states

//STATES
class StObserve;
class StLeakyLungStep1;
class StLeakyLungStep2;
class StLeakyLungStep3;
class StPatientObstructionStep1;
class StPatientObstructionStep2;
class StCalibrationStep1;
class StSystemShutdown;

//MODE STATES
class MsRun;
class MsLeakyLung;
class MsPatientObstruction;
class MsCalibration;
class MsShutdown;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmRespira1 : public smacc::SmaccStateMachineBase<SmRespira1, MsRun>
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
}  // namespace sm_respira_1

// MODE STATES
#include <sm_respira_1/mode_states/ms_calibration.h>
#include <sm_respira_1/mode_states/ms_leaky_lung.h>
#include <sm_respira_1/mode_states/ms_patient_obstruction.h>
#include <sm_respira_1/mode_states/ms_run.h>
#include <sm_respira_1/mode_states/ms_shutdown.h>

//STATES
#include <sm_respira_1/states/ms_calibration_inner_states/st_calibration_step_1.h>
#include <sm_respira_1/states/ms_leaky_lung_inner_states/st_leaky_lung_step_1.h>
#include <sm_respira_1/states/ms_leaky_lung_inner_states/st_leaky_lung_step_2.h>
#include <sm_respira_1/states/ms_leaky_lung_inner_states/st_leaky_lung_step_3.h>
#include <sm_respira_1/states/ms_patient_obstruction_inner_states/st_patient_obstruction_step_1.h>
#include <sm_respira_1/states/ms_patient_obstruction_inner_states/st_patient_obstruction_step_2.h>
#include <sm_respira_1/states/ms_shutdown_inner_states/st_system_shutdown.h>
#include <sm_respira_1/states/st_observe.h>

#include <sm_respira_1/superstates/ss_ac_cycle.h>
#include <sm_respira_1/superstates/ss_cmv_cycle.h>
#include <sm_respira_1/superstates/ss_pc_cycle.h>
#include <sm_respira_1/superstates/ss_ps_cycle.h>

//ss_ac_cycle
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_dwell.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_expire.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_inspire.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_loop.h>
#include <sm_respira_1/states/ac_cycle_inner_states/sti_ac_cycle_plateau.h>

//ss_cmv_cycle
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_dwell.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_expire.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_inspire.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_loop.h>
#include <sm_respira_1/states/cmv_cycle_inner_states/sti_cmv_cycle_plateau.h>

//ss_pc_cycle
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_dwell.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_expire.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_inspire.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_loop.h>
#include <sm_respira_1/states/pc_cycle_inner_states/sti_pc_cycle_plateau.h>

//ss_ps_cycle
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_dwell.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_expire.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_inspire.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_loop.h>
#include <sm_respira_1/states/ps_cycle_inner_states/sti_ps_cycle_plateau.h>
