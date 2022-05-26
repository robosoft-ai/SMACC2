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

class SsStartSequenceA;
namespace start_sequence_a
{
class StiStartSequenceALoop;
class StiStartSequenceAStep1;
class StiStartSequenceAStep2;
class StiStartSequenceAStep3;
class StiStartSequenceAStep4;
class StiStartSequenceAStep5;
class StiStartSequenceAStep6;
class StiStartSequenceAStep7;
class StiStartSequenceAStep8;
class StiStartSequenceAStep9;
}  // namespace start_sequence_a

class SsCompletingSequenceA;
namespace completing_sequence_a
{
class StiCompletingSequenceALoop;
class StiCompletingSequenceAStep1;
class StiCompletingSequenceAStep2;
class StiCompletingSequenceAStep3;
class StiCompletingSequenceAStep4;
class StiCompletingSequenceAStep5;
class StiCompletingSequenceAStep6;
class StiCompletingSequenceAStep7;
class StiCompletingSequenceAStep8;
class StiCompletingSequenceAStep9;
}  // namespace completing_sequence_a

class SsSuspendingSequenceA;
namespace suspending_sequence_a
{
class StiSuspendingSequenceALoop;
class StiSuspendingSequenceAStep1;
class StiSuspendingSequenceAStep2;
class StiSuspendingSequenceAStep3;
class StiSuspendingSequenceAStep4;
class StiSuspendingSequenceAStep5;
class StiSuspendingSequenceAStep6;
class StiSuspendingSequenceAStep7;
class StiSuspendingSequenceAStep8;
class StiSuspendingSequenceAStep9;
}  // namespace suspending_sequence_a

class SsSuspendingSequenceC;
namespace suspending_sequence_c
{
class StiSuspendingSequenceCLoop;
class StiSuspendingSequenceCStep1;
class StiSuspendingSequenceCStep2;
class StiSuspendingSequenceCStep3;
class StiSuspendingSequenceCStep4;
class StiSuspendingSequenceCStep5;
class StiSuspendingSequenceCStep6;
class StiSuspendingSequenceCStep7;
class StiSuspendingSequenceCStep8;
class StiSuspendingSequenceCStep9;
}  // namespace suspending_sequence_c

class SsSuspendingSequenceD;
namespace suspending_sequence_d
{
class StiSuspendingSequenceDLoop;
class StiSuspendingSequenceDStep1;
class StiSuspendingSequenceDStep2;
class StiSuspendingSequenceDStep3;
class StiSuspendingSequenceDStep4;
class StiSuspendingSequenceDStep5;
class StiSuspendingSequenceDStep6;
class StiSuspendingSequenceDStep7;
class StiSuspendingSequenceDStep8;
class StiSuspendingSequenceDStep9;
}  // namespace suspending_sequence_d

class SsHoldingSequenceA;
namespace holding_sequence_a
{
class StiHoldingSequenceALoop;
class StiHoldingSequenceAStep1;
class StiHoldingSequenceAStep2;
class StiHoldingSequenceAStep3;
class StiHoldingSequenceAStep4;
class StiHoldingSequenceAStep5;
class StiHoldingSequenceAStep6;
class StiHoldingSequenceAStep7;
class StiHoldingSequenceAStep8;
class StiHoldingSequenceAStep9;
}  // namespace holding_sequence_a


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

class SsStartSequenceB;

namespace start_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiStartSequenceBLoop;
class StiStartSequenceBStep1;
class StiStartSequenceBStep2;
class StiStartSequenceBStep3;
class StiStartSequenceBStep4;
class StiStartSequenceBStep5;
class StiStartSequenceBStep6;
class StiStartSequenceBStep7;
class StiStartSequenceBStep8;
class StiStartSequenceBStep9;
}  // namespace start_sequence_b

class SsCompletingSequenceB;

namespace completing_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiCompletingSequenceBLoop;
class StiCompletingSequenceBStep1;
class StiCompletingSequenceBStep2;
class StiCompletingSequenceBStep3;
class StiCompletingSequenceBStep4;
class StiCompletingSequenceBStep5;
class StiCompletingSequenceBStep6;
class StiCompletingSequenceBStep7;
class StiCompletingSequenceBStep8;
class StiCompletingSequenceBStep9;
}  // namespace completing_sequence_b

class SsSuspendingSequenceB;

namespace suspending_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiSuspendingSequenceB;
class StiSuspendingSequenceBStep1;
class StiSuspendingSequenceBStep2;
class StiSuspendingSequenceBStep3;
class StiSuspendingSequenceBStep4;
class StiSuspendingSequenceBStep5;
class StiSuspendingSequenceBStep6;
class StiSuspendingSequenceBStep7;
class StiSuspendingSequenceBStep8;
class StiSuspendingSequenceBStep9;
}  // namespace suspending_sequence_b

class SsHoldingSequenceB;

namespace holding_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiHoldingSequenceBLoop;
class StiHoldingSequenceBStep1;
class StiHoldingSequenceBStep2;
class StiHoldingSequenceBStep3;
class StiHoldingSequenceBStep4;
class StiHoldingSequenceBStep5;
class StiHoldingSequenceBStep6;
class StiHoldingSequenceBStep7;
class StiHoldingSequenceBStep8;
class StiHoldingSequenceBStep9;
}  // namespace holding_sequence_b

//STATES
class ExecuteStObserve;
class StRecoveryAnalyze1;
class StRecoveryBifurcate1;
class StRecoveryCalculate1;
class StRecoveryDeliberate1;
class StRecoveryEvaluate1;
class StRecoveryGenerate1;
class StRecoveryInnervate1;

class StartStObserve;
class StRecoveryAnalyze2;
class StRecoveryBifurcate2;
class StRecoveryCalculate2;
class StRecoveryDeliberate2;
class StRecoveryEvaluate2;
class StRecoveryGenerate2;
class StRecoveryInnervate2;

class CompletingStObserve;
class SuspendingStObserve;
class HoldingStObserve;


class ExecuteSequenceALoop;
class ExecuteSequenceBLoop;
class StartSequenceALoop;
class StartSequenceBLoop;
class CompletingSequenceALoop;
class CompletingSequenceBLoop;
class SuspendingSequenceALoop;
class SuspendingSequenceCLoop;
class SuspendingSequenceDLoop;
class SuspendingSequenceBLoop;
class HoldingSequenceALoop;
class HoldingSequenceBLoop;


//MODE STATES
class MsExecute;
class MsRecovery1;

//MODE STATES
class MsStarting;
class MsRecovery2;

class MsCompleting;
class MsSuspending;
class MsHolding;

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
