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
#include <sm_pack_ml/orthogonals/or_keyboard.hpp>
#include <sm_pack_ml/orthogonals/or_subscriber.hpp>
#include <sm_pack_ml/orthogonals/or_timer.hpp>

using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace sm_pack_ml::cl_subscriber;

//CLIENT BEHAVIORS
#include <sm_pack_ml/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.hpp>
#include <sm_pack_ml/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.hpp>

#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <cl_ros2_timer/client_behaviors/cb_ros_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_pack_ml
{
struct JsAborting;
struct JsActive;
struct JsAborted;

struct DsClearing;
struct DsRun;
struct DsStopped;
struct DsStopping;

struct MsComplete;
struct MsCompleting;
struct MsExecute;
struct MsHeld;
struct MsHolding;
struct MsIdle;
struct MsResetting;
struct MsStarting;
struct MsSuspended;
struct MsSuspending;
struct MsUnholding;
struct MsUnsuspending;

//SUPERSTATES
struct SsExecuteSequenceA;
namespace execute_sequence_a
{
struct StiExecuteSequenceALoop;
struct StiExecuteSequenceAStep1;
struct StiExecuteSequenceAStep2;
struct StiExecuteSequenceAStep3;
struct StiExecuteSequenceAStep4;
struct StiExecuteSequenceAStep5;
struct StiExecuteSequenceAStep6;
struct StiExecuteSequenceAStep7;
struct StiExecuteSequenceAStep8;
struct StiExecuteSequenceAStep9;
}  // namespace execute_sequence_a

struct SsStartSequenceA;
namespace start_sequence_a
{
struct StiStartSequenceALoop;
struct StiStartSequenceAStep1;
struct StiStartSequenceAStep2;
struct StiStartSequenceAStep3;
struct StiStartSequenceAStep4;
struct StiStartSequenceAStep5;
struct StiStartSequenceAStep6;
struct StiStartSequenceAStep7;
struct StiStartSequenceAStep8;
struct StiStartSequenceAStep9;
}  // namespace start_sequence_a

struct SsCompletingSequenceA;
namespace completing_sequence_a
{
struct StiCompletingSequenceALoop;
struct StiCompletingSequenceAStep1;
struct StiCompletingSequenceAStep2;
struct StiCompletingSequenceAStep3;
struct StiCompletingSequenceAStep4;
struct StiCompletingSequenceAStep5;
struct StiCompletingSequenceAStep6;
struct StiCompletingSequenceAStep7;
struct StiCompletingSequenceAStep8;
struct StiCompletingSequenceAStep9;
}  // namespace completing_sequence_a

struct SsSuspendingSequenceA;
namespace suspending_sequence_a
{
struct StiSuspendingSequenceALoop;
struct StiSuspendingSequenceAStep1;
struct StiSuspendingSequenceAStep2;
struct StiSuspendingSequenceAStep3;
struct StiSuspendingSequenceAStep4;
struct StiSuspendingSequenceAStep5;
struct StiSuspendingSequenceAStep6;
struct StiSuspendingSequenceAStep7;
struct StiSuspendingSequenceAStep8;
struct StiSuspendingSequenceAStep9;
}  // namespace suspending_sequence_a

struct SsSuspendingSequenceC;
namespace suspending_sequence_c
{
struct StiSuspendingSequenceCLoop;
struct StiSuspendingSequenceCStep1;
struct StiSuspendingSequenceCStep2;
struct StiSuspendingSequenceCStep3;
struct StiSuspendingSequenceCStep4;
struct StiSuspendingSequenceCStep5;
struct StiSuspendingSequenceCStep6;
struct StiSuspendingSequenceCStep7;
struct StiSuspendingSequenceCStep8;
struct StiSuspendingSequenceCStep9;
}  // namespace suspending_sequence_c

struct SsSuspendingSequenceD;
namespace suspending_sequence_d
{
struct StiSuspendingSequenceDLoop;
struct StiSuspendingSequenceDStep1;
struct StiSuspendingSequenceDStep2;
struct StiSuspendingSequenceDStep3;
struct StiSuspendingSequenceDStep4;
struct StiSuspendingSequenceDStep5;
struct StiSuspendingSequenceDStep6;
struct StiSuspendingSequenceDStep7;
struct StiSuspendingSequenceDStep8;
struct StiSuspendingSequenceDStep9;
}  // namespace suspending_sequence_d

struct SsHoldingSequenceA;
namespace holding_sequence_a
{
struct StiHoldingSequenceALoop;
struct StiHoldingSequenceAStep1;
struct StiHoldingSequenceAStep2;
struct StiHoldingSequenceAStep3;
struct StiHoldingSequenceAStep4;
struct StiHoldingSequenceAStep5;
struct StiHoldingSequenceAStep6;
struct StiHoldingSequenceAStep7;
struct StiHoldingSequenceAStep8;
struct StiHoldingSequenceAStep9;
}  // namespace holding_sequence_a


struct SsExecuteSequenceB;
namespace execute_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiExecuteSequenceBLoop;
struct StiExecuteSequenceBStep1;
struct StiExecuteSequenceBStep2;
struct StiExecuteSequenceBStep3;
struct StiExecuteSequenceBStep4;
struct StiExecuteSequenceBStep5;
struct StiExecuteSequenceBStep6;
struct StiExecuteSequenceBStep7;
struct StiExecuteSequenceBStep8;
struct StiExecuteSequenceBStep9;
}  // namespace execute_sequence_b

struct SsStartSequenceB;

namespace start_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiStartSequenceBLoop;
struct StiStartSequenceBStep1;
struct StiStartSequenceBStep2;
struct StiStartSequenceBStep3;
struct StiStartSequenceBStep4;
struct StiStartSequenceBStep5;
struct StiStartSequenceBStep6;
struct StiStartSequenceBStep7;
struct StiStartSequenceBStep8;
struct StiStartSequenceBStep9;
}  // namespace start_sequence_b

struct SsCompletingSequenceB;

namespace completing_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiCompletingSequenceBLoop;
struct StiCompletingSequenceBStep1;
struct StiCompletingSequenceBStep2;
struct StiCompletingSequenceBStep3;
struct StiCompletingSequenceBStep4;
struct StiCompletingSequenceBStep5;
struct StiCompletingSequenceBStep6;
struct StiCompletingSequenceBStep7;
struct StiCompletingSequenceBStep8;
struct StiCompletingSequenceBStep9;
}  // namespace completing_sequence_b

struct SsSuspendingSequenceB;

namespace suspending_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiSuspendingSequenceB;
struct StiSuspendingSequenceBStep1;
struct StiSuspendingSequenceBStep2;
struct StiSuspendingSequenceBStep3;
struct StiSuspendingSequenceBStep4;
struct StiSuspendingSequenceBStep5;
struct StiSuspendingSequenceBStep6;
struct StiSuspendingSequenceBStep7;
struct StiSuspendingSequenceBStep8;
struct StiSuspendingSequenceBStep9;
}  // namespace suspending_sequence_b

struct SsHoldingSequenceB;

namespace holding_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiHoldingSequenceBLoop;
struct StiHoldingSequenceBStep1;
struct StiHoldingSequenceBStep2;
struct StiHoldingSequenceBStep3;
struct StiHoldingSequenceBStep4;
struct StiHoldingSequenceBStep5;
struct StiHoldingSequenceBStep6;
struct StiHoldingSequenceBStep7;
struct StiHoldingSequenceBStep8;
struct StiHoldingSequenceBStep9;
}  // namespace holding_sequence_b

//STATES
struct ExecuteStObserve;
struct StRecoveryAnalyze1;
struct StRecoveryBifurcate1;
struct StRecoveryCalculate1;
struct StRecoveryDeliberate1;
struct StRecoveryEvaluate1;
struct StRecoveryGenerate1;
struct StRecoveryInnervate1;

struct StartStObserve;
struct StRecoveryAnalyze2;
struct StRecoveryBifurcate2;
struct StRecoveryCalculate2;
struct StRecoveryDeliberate2;
struct StRecoveryEvaluate2;
struct StRecoveryGenerate2;
struct StRecoveryInnervate2;

struct CompletingStObserve;
struct SuspendingStObserve;
struct HoldingStObserve;


struct ExecuteSequenceALoop;
struct ExecuteSequenceBLoop;
struct StartSequenceALoop;
struct StartSequenceBLoop;
struct CompletingSequenceALoop;
struct CompletingSequenceBLoop;
struct SuspendingSequenceALoop;
struct SuspendingSequenceCLoop;
struct SuspendingSequenceDLoop;
class SuspendingSequenceBLoop;
struct HoldingSequenceALoop;
struct HoldingSequenceBLoop;


//MODE STATES
struct MsExecute;
class MsRecovery1;

//MODE STATES
struct MsStarting;
class MsRecovery2;

struct MsCompleting;
struct MsSuspending;
struct MsHolding;

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

struct EvClear:sc::event<EvClear>
{
};

// STATE MACHINE
struct SmPackMl1 : public smacc2::SmaccStateMachineBase<SmPackMl1, JsActive>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
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
#include "states/start_sequence_a_loop.hpp"

#include <sm_pack_ml/states/completing_sequence_a_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_a_loop.hpp>
#include <sm_pack_ml/states/holding_sequence_a_loop.hpp>

#include <sm_pack_ml/states/suspending_sequence_c_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_d_loop.hpp>

#include <sm_pack_ml/states/execute_sequence_b_loop.hpp>
#include <sm_pack_ml/states/start_sequence_b_loop.hpp>
#include <sm_pack_ml/states/completing_sequence_b_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_b_loop.hpp>
#include <sm_pack_ml/states/holding_sequence_b_loop.hpp>

#include <sm_pack_ml/states/execute_st_observe.hpp>
#include <sm_pack_ml/states/start_st_observe.hpp>
#include <sm_pack_ml/states/completing_st_observe.hpp>
#include <sm_pack_ml/states/suspending_st_observe.hpp>
#include <sm_pack_ml/states/holding_st_observe.hpp>

#include <sm_pack_ml/superstates/ss_execute_sequence_a.hpp>
#include <sm_pack_ml/superstates/ss_start_sequence_a.hpp>
#include <sm_pack_ml/superstates/ss_completing_sequence_a.hpp>
#include <sm_pack_ml/superstates/ss_suspending_sequence_a.hpp>
#include <sm_pack_ml/superstates/ss_holding_sequence_a.hpp>

#include <sm_pack_ml/superstates/ss_suspending_sequence_c.hpp>
#include <sm_pack_ml/superstates/ss_suspending_sequence_d.hpp>

#include <sm_pack_ml/superstates/ss_execute_sequence_b.hpp>
#include <sm_pack_ml/superstates/ss_start_sequence_b.hpp>
#include <sm_pack_ml/superstates/ss_completing_sequence_b.hpp>
#include <sm_pack_ml/superstates/ss_suspending_sequence_b.hpp>
#include <sm_pack_ml/superstates/ss_holding_sequence_b.hpp>

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

// //ss_start_sequence_a
#include "states/start_sequence_a/sti_start_sequence_a_step_4.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_3.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_1.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_loop.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_2.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_5.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_9.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_8.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_7.hpp"
#include "states/start_sequence_a/sti_start_sequence_a_step_6.hpp"

//ss_completing_sequence_a
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_4.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_3.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_1.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_loop.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_2.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_5.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_9.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_8.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_7.hpp>
#include <sm_pack_ml/states/completing_sequence_a/sti_completing_sequence_a_step_6.hpp>

//ss_suspending_sequence_a
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_4.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_3.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_1.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_2.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_5.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_9.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_8.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_7.hpp>
#include <sm_pack_ml/states/suspending_sequence_a/sti_suspending_sequence_a_step_6.hpp>

//ss_suspending_sequence_c
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_1.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_2.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_3.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_4.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_5.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_6.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_7.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_8.hpp>
#include <sm_pack_ml/states/suspending_sequence_c/sti_suspending_sequence_c_step_9.hpp>

//ss_suspending_sequence_d
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_4.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_3.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_1.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_2.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_5.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_9.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_8.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_7.hpp>
#include <sm_pack_ml/states/suspending_sequence_d/sti_suspending_sequence_d_step_6.hpp>

//ss_holding_sequence_a
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_4.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_3.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_1.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_loop.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_2.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_5.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_9.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_8.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_7.hpp>
#include <sm_pack_ml/states/holding_sequence_a/sti_holding_sequence_a_step_6.hpp>

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
#include "states/start_sequence_b/sti_start_sequence_b_step_4.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_3.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_1.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_loop.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_2.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_5.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_9.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_8.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_7.hpp"
#include "states/start_sequence_b/sti_start_sequence_b_step_6.hpp"

//ss_completing_sequence_b
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_4.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_3.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_1.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_loop.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_2.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_5.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_9.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_8.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_7.hpp>
#include <sm_pack_ml/states/completing_sequence_b/sti_completing_sequence_b_step_6.hpp>

//ss_suspending_sequence_b
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_4.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_3.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_1.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_loop.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_2.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_5.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_9.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_8.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_7.hpp>
#include <sm_pack_ml/states/suspending_sequence_b/sti_suspending_sequence_b_step_6.hpp>

//ss_holding_seqence_b
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_4.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_3.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_1.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_loop.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_2.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_5.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_9.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_8.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_7.hpp>
#include <sm_pack_ml/states/holding_sequence_b/sti_holding_sequence_b_step_6.hpp>
