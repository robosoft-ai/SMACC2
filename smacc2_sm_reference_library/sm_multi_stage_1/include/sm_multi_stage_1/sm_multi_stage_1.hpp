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
class SsASequence1;
namespace a_sequence_1
{
class StiASequenceLoop1;
class StiASequenceInspire1;
class StiASequencePlateau1;
class StiASequenceExpire1;
class StiASequenceDwell1;
class StiASequencePulse1;
class StiASequenceTitrate1;
class StiASequenceRinse1;
class StiASequenceRecycle1;
class StiASequencePush1;
}  // namespace a_sequence_1

class SsASequence2;
namespace a_sequence_2
{
class StiASequenceLoop2;
class StiASequenceInspire2;
class StiASequencePlateau2;
class StiASequenceExpire2;
class StiASequenceDwell2;
class StiASequencePulse2;
class StiASequenceTitrate2;
class StiASequenceRinse2;
class StiASequenceRecycle2;
class StiASequencePush2;
}  // namespace a_sequence_2

class SsASequence3;
namespace a_sequence_3
{
class StiASequenceLoop3;
class StiASequenceInspire3;
class StiASequencePlateau3;
class StiASequenceExpire3;
class StiASequenceDwell3;
class StiASequencePulse3;
class StiASequenceTitrate3;
class StiASequenceRinse3;
class StiASequenceRecycle3;
class StiASequencePush3;
}  // namespace a_sequence_3

class SsASequence4;
namespace a_sequence_4
{
class StiASequenceLoop4;
class StiASequenceInspire4;
class StiASequencePlateau4;
class StiASequenceExpire4;
class StiASequenceDwell4;
class StiASequencePulse4;
class StiASequenceTitrate4;
class StiASequenceRinse4;
class StiASequenceRecycle4;
class StiASequencePush4;
}  // namespace a_sequence_4

class SsCSequence4;
namespace c_sequence_4
{
class StiCSequenceLoop4;
class StiCSequenceInspire4;
class StiCSequencePlateau4;
class StiCSequenceExpire4;
class StiCSequenceDwell4;
class StiCSequencePulse4;
class StiCSequenceTitrate4;
class StiCSequenceRinse4;
class StiCSequenceRecycle4;
class StiCSequencePush4;
}  // namespace c_sequence_4

class SsDSequence4;
namespace d_sequence_4
{
class StiDSequenceLoop4;
class StiDSequenceInspire4;
class StiDSequencePlateau4;
class StiDSequenceExpire4;
class StiDSequenceDwell4;
class StiDSequencePulse4;
class StiDSequenceTitrate4;
class StiDSequenceRinse4;
class StiDSequenceRecycle4;
class StiDSequencePush4;
}  // namespace d_sequence_4

class SsASequence5;
namespace a_sequence_5
{
class StiASequenceLoop5;
class StiASequenceInspire5;
class StiASequencePlateau5;
class StiASequenceExpire5;
class StiASequenceDwell5;
class StiASequencePulse5;
class StiASequenceTitrate5;
class StiASequenceRinse5;
class StiASequenceRecycle5;
class StiASequencePush5;
}  // namespace a_sequence_5


class SsBSequence1;

namespace b_sequence_1
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop1;
class StiBSequenceInspire1;
class StiBSequencePlateau1;
class StiBSequenceExpire1;
class StiBSequenceDwell1;
class StiBSequencePulse1;
class StiBSequenceTitrate1;
class StiBSequenceRinse1;
class StiBSequenceRecycle1;
class StiBSequencePush1;
}  // namespace b_sequence_1

class SsBSequence2;

namespace b_sequence_2
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop2;
class StiBSequenceInspire2;
class StiBSequencePlateau2;
class StiBSequenceExpire2;
class StiBSequenceDwell2;
class StiBSequencePulse2;
class StiBSequenceTitrate2;
class StiBSequenceRinse2;
class StiBSequenceRecycle2;
class StiBSequencePush2;
}  // namespace b_sequence_2

class SsBSequence3;

namespace b_sequence_3
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop3;
class StiBSequenceInspire3;
class StiBSequencePlateau3;
class StiBSequenceExpire3;
class StiBSequenceDwell3;
class StiBSequencePulse3;
class StiBSequenceTitrate3;
class StiBSequenceRinse3;
class StiBSequenceRecycle3;
class StiBSequencePush3;
}  // namespace b_sequence_3

class SsBSequence4;

namespace b_sequence_4
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop4;
class StiBSequenceInspire4;
class StiBSequencePlateau4;
class StiBSequenceExpire4;
class StiBSequenceDwell4;
class StiBSequencePulse4;
class StiBSequenceTitrate4;
class StiBSequenceRinse4;
class StiBSequenceRecycle4;
class StiBSequencePush4;
}  // namespace b_sequence_4

class SsBSequence5;

namespace b_sequence_5
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop5;
class StiBSequenceInspire5;
class StiBSequencePlateau5;
class StiBSequenceExpire5;
class StiBSequenceDwell5;
class StiBSequencePulse5;
class StiBSequenceTitrate5;
class StiBSequenceRinse5;
class StiBSequenceRecycle5;
class StiBSequencePush5;
}  // namespace b_sequence_5

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


class ASequenceLoop1;
class BSequenceLoop1;
class ASequenceLoop2;
class BSequenceLoop2;
class ASequenceLoop3;
class BSequenceLoop3;
class ASequenceLoop4;
class CSequenceLoop4;
class DSequenceLoop4;
class BSequenceLoop4;
class ASequenceLoop5;
class BSequenceLoop5;


//MODE STATES
class MsMode1;
class MsRecovery1;

//MODE STATES
class MsMode2;
class MsRecovery2;

class MsMode3;
class MsMode4;
class MsMode5;

struct EvToDeep : sc::event<EvToDeep>
{
};
struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmMultiStage1 : public smacc2::SmaccStateMachineBase<SmMultiStage1, MsMode1>
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
#include <sm_multi_stage_1/mode_states/ms_mode_1.hpp>
#include <sm_multi_stage_1/mode_states/ms_mode_2.hpp>
#include <sm_multi_stage_1/mode_states/ms_mode_3.hpp>
#include <sm_multi_stage_1/mode_states/ms_mode_4.hpp>
#include <sm_multi_stage_1/mode_states/ms_mode_5.hpp>

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

#include <sm_multi_stage_1/states/a_sequence_loop_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_loop_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_loop_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_loop_5.hpp>

#include <sm_multi_stage_1/states/c_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_loop_4.hpp>

#include <sm_multi_stage_1/states/b_sequence_loop_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_loop_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_loop_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_loop_5.hpp>

#include <sm_multi_stage_1/states/st_observe_1.hpp>
#include <sm_multi_stage_1/states/st_observe_2.hpp>
#include <sm_multi_stage_1/states/st_observe_3.hpp>
#include <sm_multi_stage_1/states/st_observe_4.hpp>
#include <sm_multi_stage_1/states/st_observe_5.hpp>

#include <sm_multi_stage_1/superstates/ss_a_sequence_1.hpp>
#include <sm_multi_stage_1/superstates/ss_a_sequence_2.hpp>
#include <sm_multi_stage_1/superstates/ss_a_sequence_3.hpp>
#include <sm_multi_stage_1/superstates/ss_a_sequence_4.hpp>
#include <sm_multi_stage_1/superstates/ss_a_sequence_5.hpp>

#include <sm_multi_stage_1/superstates/ss_c_sequence_4.hpp>
#include <sm_multi_stage_1/superstates/ss_d_sequence_4.hpp>


#include <sm_multi_stage_1/superstates/ss_b_sequence_1.hpp>
#include <sm_multi_stage_1/superstates/ss_b_sequence_2.hpp>
#include <sm_multi_stage_1/superstates/ss_b_sequence_3.hpp>
#include <sm_multi_stage_1/superstates/ss_b_sequence_4.hpp>
#include <sm_multi_stage_1/superstates/ss_b_sequence_5.hpp>


//ss_a_sequence_1
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_dwell_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_expire_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_inspire_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_loop_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_plateau_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_pulse_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_push_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_recycle_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_rinse_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_titrate_1.hpp>

//ss_a_sequence_2
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_dwell_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_expire_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_inspire_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_loop_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_plateau_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_pulse_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_push_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_recycle_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_rinse_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_titrate_2.hpp>

//ss_a_sequence_3
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_dwell_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_expire_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_inspire_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_loop_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_plateau_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_pulse_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_push_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_recycle_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_rinse_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_titrate_3.hpp>

//ss_a_sequence_4
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_dwell_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_expire_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_inspire_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_plateau_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_pulse_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_push_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_recycle_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_rinse_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_titrate_4.hpp>

//ss_c_sequence_4
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_dwell_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_expire_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_inspire_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_plateau_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_pulse_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_push_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_recycle_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_rinse_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_titrate_4.hpp>

//ss_d_sequence_4
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_dwell_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_expire_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_inspire_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_plateau_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_pulse_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_push_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_recycle_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_rinse_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_titrate_4.hpp>

//ss_a_sequence_5
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_dwell_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_expire_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_inspire_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_loop_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_plateau_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_pulse_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_push_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_recycle_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_rinse_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_titrate_5.hpp>

//ss_b_sequence_1
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_dwell_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_expire_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_inspire_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_loop_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_plateau_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_pulse_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_push_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_recycle_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_rinse_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_titrate_1.hpp>

//ss_b_sequence_2
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_dwell_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_expire_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_inspire_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_loop_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_plateau_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_pulse_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_push_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_recycle_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_rinse_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_titrate_2.hpp>

//ss_b_sequence_3
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_dwell_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_expire_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_inspire_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_loop_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_plateau_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_pulse_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_push_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_recycle_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_rinse_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_titrate_3.hpp>

//ss_b_sequence_4
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_dwell_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_expire_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_inspire_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_plateau_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_pulse_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_push_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_recycle_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_rinse_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_titrate_4.hpp>

//ss_b_sequence_5
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_dwell_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_expire_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_inspire_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_loop_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_plateau_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_pulse_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_push_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_recycle_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_rinse_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_titrate_5.hpp>
