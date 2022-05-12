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
#include <sm_pack_ml/orthogonals/or_keyboard.hpp>
#include <sm_pack_ml/orthogonals/or_subscriber.hpp>
#include <sm_pack_ml/orthogonals/or_timer.hpp>
#include <sm_pack_ml/orthogonals/or_updatable_publisher.hpp>

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
using namespace cl_keyboard;
using namespace sm_pack_ml::cl_subscriber;

//CLIENT BEHAVIORS
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

#include <sm_pack_ml/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.hpp>
#include <sm_pack_ml/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.hpp>

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_pack_ml
{
class JsAborting;
class JsActive;
class JsAborted;

class DsClearing;
class DsRun;
class DsStopped;
class DsStopping;

class MsComplete;
class MsCompleting;
class MsExecute;
class MsHeld;
class MsHolding;
class MsIdle;
class MsResetting;
class MsStarting;
class MsSuspended;
class MsSuspending;
class MsUnholding;
class MsUnsuspending;

//SUPERSTATES
class SsExecuteSequenceA;
namespace execute_sequence_a
{
class StiExecuteSequenceALoop;
class StiExecuteSequenceAStep1;
class StiExecuteSequenceAStep2;
class StiExecuteSequenceAStep3;
class StiExecuteSequenceAStep4;
class StiExecuteSequenceAStep5;
class StiExecuteSequenceAStep6;
class StiExecuteSequenceAStep7;
class StiExecuteSequenceAStep8;
class StiExecuteSequenceAStep9;
}  // namespace execute_sequence_a

class SsMode2SequenceA;
namespace mode_2_sequence_a
{
class StiMode2SequenceALoop;
class StiMode2SequenceAStep1;
class StiMode2SequenceAStep2;
class StiMode2SequenceAStep3;
class StiMode2SequenceAStep4;
class StiMode2SequenceAStep5;
class StiMode2SequenceAStep6;
class StiMode2SequenceAStep7;
class StiMode2SequenceAStep8;
class StiMode2SequenceAStep9;
}  // namespace mode_2_sequence_a

class SsMode3SequenceA;
namespace mode_3_sequence_a
{
class StiMode3SequenceALoop;
class StiMode3SequenceAStep1;
class StiMode3SequenceAStep2;
class StiMode3SequenceAStep3;
class StiMode3SequenceAStep4;
class StiMode3SequenceAStep5;
class StiMode3SequenceAStep6;
class StiMode3SequenceAStep7;
class StiMode3SequenceAStep8;
class StiMode3SequenceAStep9;
}  // namespace mode_3_sequence_a

class SsMode4SequenceA;
namespace mode_4_sequence_a
{
class StiMode4SequenceALoop;
class StiMode4SequenceAStep1;
class StiMode4SequenceAStep2;
class StiMode4SequenceAStep3;
class StiMode4SequenceAStep4;
class StiMode4SequenceAStep5;
class StiMode4SequenceAStep6;
class StiMode4SequenceAStep7;
class StiMode4SequenceAStep8;
class StiMode4SequenceAStep9;
}  // namespace mode_4_sequence_a

class SsMode4SequenceC;
namespace mode_4_sequence_c
{
class StiMode4SequenceCLoop;
class StiMode4SequenceCStep1;
class StiMode4SequenceCStep2;
class StiMode4SequenceCStep3;
class StiMode4SequenceCStep4;
class StiMode4SequenceCStep5;
class StiMode4SequenceCStep6;
class StiMode4SequenceCStep7;
class StiMode4SequenceCStep8;
class StiMode4SequenceCStep9;
}  // namespace mode_4_sequence_c

class SsMode4SequenceD;
namespace mode_4_sequence_d
{
class StiMode4SequenceDLoop;
class StiMode4SequenceDStep1;
class StiMode4SequenceDStep2;
class StiMode4SequenceDStep3;
class StiMode4SequenceDStep4;
class StiMode4SequenceDStep5;
class StiMode4SequenceDStep6;
class StiMode4SequenceDStep7;
class StiMode4SequenceDStep8;
class StiMode4SequenceDStep9;
}  // namespace mode_4_sequence_d

class SsMode5SequenceA;
namespace mode_5_sequence_a
{
class StiMode5SequenceALoop;
class StiMode5SequenceAStep1;
class StiMode5SequenceAStep2;
class StiMode5SequenceAStep3;
class StiMode5SequenceAStep4;
class StiMode5SequenceAStep5;
class StiMode5SequenceAStep6;
class StiMode5SequenceAStep7;
class StiMode5SequenceAStep8;
class StiMode5SequenceAStep9;
}  // namespace mode_5_sequence_a


class SsExecuteSequenceB;
namespace execute_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiExecuteSequenceBLoop;
class StiExecuteSequenceBStep1;
class StiExecuteSequenceBStep2;
class StiExecuteSequenceBStep3;
class StiExecuteSequenceBStep4;
class StiExecuteSequenceBStep5;
class StiExecuteSequenceBStep6;
class StiExecuteSequenceBStep7;
class StiExecuteSequenceBStep8;
class StiExecuteSequenceBStep9;
}  // namespace execute_sequence_b

class SsMode2SequenceB;

namespace mode_2_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiMode2SequenceBLoop;
class StiMode2SequenceBStep1;
class StiMode2SequenceBStep2;
class StiMode2SequenceBStep3;
class StiMode2SequenceBStep4;
class StiMode2SequenceBStep5;
class StiMode2SequenceBStep6;
class StiMode2SequenceBStep7;
class StiMode2SequenceBStep8;
class StiMode2SequenceBStep9;
}  // namespace mode_2_sequence_b

class SsMode3SequenceB;

namespace mode_3_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiMode3SequenceBLoop;
class StiMode3SequenceBStep1;
class StiMode3SequenceBStep2;
class StiMode3SequenceBStep3;
class StiMode3SequenceBStep4;
class StiMode3SequenceBStep5;
class StiMode3SequenceBStep6;
class StiMode3SequenceBStep7;
class StiMode3SequenceBStep8;
class StiMode3SequenceBStep9;
}  // namespace mode_3_sequence_b

class SsMode4SequenceB;

namespace mode_4_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiMode4SequenceB;
class StiMode4SequenceBStep1;
class StiMode4SequenceBStep2;
class StiMode4SequenceBStep3;
class StiMode4SequenceBStep4;
class StiMode4SequenceBStep5;
class StiMode4SequenceBStep6;
class StiMode4SequenceBStep7;
class StiMode4SequenceBStep8;
class StiMode4SequenceBStep9;
}  // namespace mode_4_sequence_b

class SsMode5SequenceB;

namespace mode_5_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiMode5SequenceBLoop;
class StiMode5SequenceBStep1;
class StiMode5SequenceBStep2;
class StiMode5SequenceBStep3;
class StiMode5SequenceBStep4;
class StiMode5SequenceBStep5;
class StiMode5SequenceBStep6;
class StiMode5SequenceBStep7;
class StiMode5SequenceBStep8;
class StiMode5SequenceBStep9;
}  // namespace mode_5_sequence_b

//STATES
class ExecuteStObserve;
class StRecoveryAnalyze1;
class StRecoveryBifurcate1;
class StRecoveryCalculate1;
class StRecoveryDeliberate1;
class StRecoveryEvaluate1;
class StRecoveryGenerate1;
class StRecoveryInnervate1;

class Mode2StObserve;
class StRecoveryAnalyze2;
class StRecoveryBifurcate2;
class StRecoveryCalculate2;
class StRecoveryDeliberate2;
class StRecoveryEvaluate2;
class StRecoveryGenerate2;
class StRecoveryInnervate2;

class Mode3StObserve;
class Mode4StObserve;
class Mode5StObserve;


class ExecuteSequenceALoop;
class ExecuteSequenceBLoop;
class Mode2SequenceALoop;
class Mode2SequenceBLoop;
class Mode3SequenceALoop;
class Mode3SequenceBLoop;
class Mode4SequenceALoop;
class Mode4SequenceCLoop;
class Mode4SequenceDLoop;
class Mode4SequenceBLoop;
class Mode5SequenceALoop;
class Mode5SequenceBLoop;


//MODE STATES
class MsExecute;
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

struct EvSc : sc::event<EvSc>
{
};

struct EvStart : sc::event<EvStart>
{
};

struct EvReset : sc::event<EvReset>
{
};

struct EvSuspend : sc::event<EvSuspend>
{
};

struct EvUnSuspend : sc::event<EvUnSuspend>
{
};

struct EvHold : sc::event<EvHold>
{
};

struct EvUnhold:sc::event<EvUnhold>
{
}; 

struct EvStop:sc::event<EvStop>
{
}; 

// STATE MACHINE
struct SmPackMl1 : public smacc2::SmaccStateMachineBase<SmPackMl1, JsActive>
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
}  // namespace sm_pack_ml

#include "j_states/js_aborted.hpp"
#include "j_states/js_aborting.hpp"
#include "j_states/js_active.hpp"

#include "domain_states/ds_clearing.hpp"
#include "domain_states/ds_run.hpp"
#include "domain_states/ds_stopped.hpp"
#include "domain_states/ds_stopping.hpp"

#include "mode_states/ms_complete.hpp"
#include "mode_states/ms_completing.hpp"
#include "mode_states/ms_execute.hpp"
#include "mode_states/ms_held.hpp"
#include "mode_states/ms_holding.hpp"
#include "mode_states/ms_idle.hpp"
#include "mode_states/ms_resetting.hpp"
#include "mode_states/ms_starting.hpp"
#include "mode_states/ms_suspended.hpp"
#include "mode_states/ms_suspending.hpp"
#include "mode_states/ms_unholding.hpp"
#include "mode_states/ms_unsuspending.hpp"

// // MODE STATES
// #include <sm_pack_ml/mode_states/ms_recovery_1.hpp>
// #include <sm_pack_ml/mode_states/ms_recovery_2.hpp>
// #include <sm_pack_ml/mode_states/ms_mode_1.hpp>
// #include <sm_pack_ml/mode_states/ms_mode_2.hpp>
// #include <sm_pack_ml/mode_states/ms_mode_3.hpp>
// #include <sm_pack_ml/mode_states/ms_mode_4.hpp>
// #include <sm_pack_ml/mode_states/ms_mode_5.hpp>

// //STATES
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_analyze_1.hpp>
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_bifurcate_1.hpp>
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_calculate_1.hpp>
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_deliberate_1.hpp>
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_evaluate_1.hpp>
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_generate_1.hpp>
// #include <sm_pack_ml/states/ms_recovery_1/st_recovery_innervate_1.hpp>

// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_analyze_2.hpp>
// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_bifurcate_2.hpp>
// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_calculate_2.hpp>
// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_deliberate_2.hpp>
// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_evaluate_2.hpp>
// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_generate_2.hpp>
// #include <sm_pack_ml/states/ms_recovery_2/st_recovery_innervate_2.hpp>

#include "states/execute_sequence_a_loop.hpp"
// #include <sm_pack_ml/states/mode_2_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a_loop.hpp>

// #include <sm_pack_ml/states/mode_4_sequence_c_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d_loop.hpp>

#include <sm_pack_ml/states/execute_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b_loop.hpp>

#include <sm_pack_ml/states/execute_st_observe.hpp>
// #include <sm_pack_ml/states/mode_2_st_observe.hpp>
// #include <sm_pack_ml/states/mode_3_st_observe.hpp>
// #include <sm_pack_ml/states/mode_4_st_observe.hpp>
// #include <sm_pack_ml/states/mode_5_st_observe.hpp>

#include <sm_pack_ml/superstates/ss_execute_sequence_a.hpp>
// #include <sm_pack_ml/superstates/ss_mode_2_sequence_a.hpp>
// #include <sm_pack_ml/superstates/ss_mode_3_sequence_a.hpp>
// #include <sm_pack_ml/superstates/ss_mode_4_sequence_a.hpp>
// #include <sm_pack_ml/superstates/ss_mode_5_sequence_a.hpp>

// #include <sm_pack_ml/superstates/ss_mode_4_sequence_c.hpp>
// #include <sm_pack_ml/superstates/ss_mode_4_sequence_d.hpp>

#include <sm_pack_ml/superstates/ss_execute_sequence_b.hpp>
// #include <sm_pack_ml/superstates/ss_mode_2_sequence_b.hpp>
// #include <sm_pack_ml/superstates/ss_mode_3_sequence_b.hpp>
// #include <sm_pack_ml/superstates/ss_mode_4_sequence_b.hpp>
// #include <sm_pack_ml/superstates/ss_mode_5_sequence_b.hpp>

// //ss_execute_sequence_a
#include "states/execute_sequence_a/sti_execute_sequence_a_step_4.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_3.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_1.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_loop.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_2.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_5.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_9.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_8.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_7.hpp"
#include "states/execute_sequence_a/sti_execute_sequence_a_step_6.hpp"

// //ss_mode_2_sequence_a
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_4.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_3.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_1.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_2.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_5.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_9.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_8.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_7.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_6.hpp>

// //ss_mode_3_sequence_a
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_4.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_3.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_1.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_2.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_5.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_9.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_8.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_7.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_6.hpp>

// //ss_mode_4_sequence_a
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_4.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_3.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_1.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_2.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_5.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_9.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_8.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_7.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_6.hpp>

// //ss_mode_4_sequence_c
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_1.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_2.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_3.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_4.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_5.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_6.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_7.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_8.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_9.hpp>

// //ss_mode_4_sequence_d
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_4.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_3.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_1.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_2.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_5.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_9.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_8.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_7.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_6.hpp>

// //ss_mode_5_sequence_a
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_4.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_3.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_1.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_loop.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_2.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_5.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_9.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_8.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_7.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_6.hpp>

// //ss_execute_sequence_b
#include "states/execute_sequence_b/sti_execute_sequence_b_step_4.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_3.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_1.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_loop.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_2.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_5.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_9.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_8.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_7.hpp"
#include "states/execute_sequence_b/sti_execute_sequence_b_step_6.hpp"

// //ss_mode_2_sequence_b
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_4.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_3.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_1.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_2.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_5.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_9.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_8.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_7.hpp>
// #include <sm_pack_ml/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_6.hpp>

// //ss_mode_3_sequence_b
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_4.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_3.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_1.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_2.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_5.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_9.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_8.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_7.hpp>
// #include <sm_pack_ml/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_6.hpp>

// //ss_mode_4_sequence_b
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_4.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_3.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_1.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_2.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_5.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_9.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_8.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_7.hpp>
// #include <sm_pack_ml/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_6.hpp>

// //ss_mode_5_seqence_b
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_4.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_3.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_1.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_loop.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_2.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_5.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_9.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_8.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_7.hpp>
// #include <sm_pack_ml/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_6.hpp>
