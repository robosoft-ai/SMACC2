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
#include <sm_multi_stage_1/orthogonals/or_keyboard.hpp>
#include <sm_multi_stage_1/orthogonals/or_timer.hpp>

using namespace cl_ros2_timer;
using namespace cl_keyboard;

//CLIENT BEHAVIORS
#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>

//#include <cl_ros2_timer/client_behaviors/cb_ros_timer.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;

namespace sm_multi_stage_1
{
//SUPERSTATES
struct SsMode1SequenceA;
namespace mode_1_sequence_a
{
struct StiMode1SequenceALoop;
struct StiMode1SequenceAStep1;
struct StiMode1SequenceAStep2;
struct StiMode1SequenceAStep3;
struct StiMode1SequenceAStep4;
struct StiMode1SequenceAStep5;
struct StiMode1SequenceAStep6;
struct StiMode1SequenceAStep7;
struct StiMode1SequenceAStep8;
struct StiMode1SequenceAStep9;
}  // namespace mode_1_sequence_a

struct SsMode2SequenceA;
namespace mode_2_sequence_a
{
struct StiMode2SequenceALoop;
struct StiMode2SequenceAStep1;
struct StiMode2SequenceAStep2;
struct StiMode2SequenceAStep3;
struct StiMode2SequenceAStep4;
struct StiMode2SequenceAStep5;
struct StiMode2SequenceAStep6;
struct StiMode2SequenceAStep7;
struct StiMode2SequenceAStep8;
struct StiMode2SequenceAStep9;
}  // namespace mode_2_sequence_a

struct SsMode3SequenceA;
namespace mode_3_sequence_a
{
struct StiMode3SequenceALoop;
struct StiMode3SequenceAStep1;
struct StiMode3SequenceAStep2;
struct StiMode3SequenceAStep3;
struct StiMode3SequenceAStep4;
struct StiMode3SequenceAStep5;
struct StiMode3SequenceAStep6;
struct StiMode3SequenceAStep7;
struct StiMode3SequenceAStep8;
struct StiMode3SequenceAStep9;
}  // namespace mode_3_sequence_a

struct SsMode4SequenceA;
namespace mode_4_sequence_a
{
struct StiMode4SequenceALoop;
struct StiMode4SequenceAStep1;
struct StiMode4SequenceAStep2;
struct StiMode4SequenceAStep3;
struct StiMode4SequenceAStep4;
struct StiMode4SequenceAStep5;
struct StiMode4SequenceAStep6;
struct StiMode4SequenceAStep7;
struct StiMode4SequenceAStep8;
struct StiMode4SequenceAStep9;
}  // namespace mode_4_sequence_a

struct SsMode4SequenceC;
namespace mode_4_sequence_c
{
struct StiMode4SequenceCLoop;
struct StiMode4SequenceCStep1;
struct StiMode4SequenceCStep2;
struct StiMode4SequenceCStep3;
struct StiMode4SequenceCStep4;
struct StiMode4SequenceCStep5;
struct StiMode4SequenceCStep6;
struct StiMode4SequenceCStep7;
struct StiMode4SequenceCStep8;
struct StiMode4SequenceCStep9;
}  // namespace mode_4_sequence_c

struct SsMode4SequenceD;
namespace mode_4_sequence_d
{
struct StiMode4SequenceDLoop;
struct StiMode4SequenceDStep1;
struct StiMode4SequenceDStep2;
struct StiMode4SequenceDStep3;
struct StiMode4SequenceDStep4;
struct StiMode4SequenceDStep5;
struct StiMode4SequenceDStep6;
struct StiMode4SequenceDStep7;
struct StiMode4SequenceDStep8;
struct StiMode4SequenceDStep9;
}  // namespace mode_4_sequence_d

struct SsMode5SequenceA;
namespace mode_5_sequence_a
{
struct StiMode5SequenceALoop;
struct StiMode5SequenceAStep1;
struct StiMode5SequenceAStep2;
struct StiMode5SequenceAStep3;
struct StiMode5SequenceAStep4;
struct StiMode5SequenceAStep5;
struct StiMode5SequenceAStep6;
struct StiMode5SequenceAStep7;
struct StiMode5SequenceAStep8;
struct StiMode5SequenceAStep9;
}  // namespace mode_5_sequence_a


struct SsMode1SequenceB;
namespace mode_1_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiMode1SequenceBLoop;
struct StiMode1SequenceBStep1;
struct StiMode1SequenceBStep2;
struct StiMode1SequenceBStep3;
struct StiMode1SequenceBStep4;
struct StiMode1SequenceBStep5;
struct StiMode1SequenceBStep6;
struct StiMode1SequenceBStep7;
struct StiMode1SequenceBStep8;
struct StiMode1SequenceBStep9;
}  // namespace mode_1_sequence_b

struct SsMode2SequenceB;

namespace mode_2_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiMode2SequenceBLoop;
struct StiMode2SequenceBStep1;
struct StiMode2SequenceBStep2;
struct StiMode2SequenceBStep3;
struct StiMode2SequenceBStep4;
struct StiMode2SequenceBStep5;
struct StiMode2SequenceBStep6;
struct StiMode2SequenceBStep7;
struct StiMode2SequenceBStep8;
struct StiMode2SequenceBStep9;
}  // namespace mode_2_sequence_b

struct SsMode3SequenceB;

namespace mode_3_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiMode3SequenceBLoop;
struct StiMode3SequenceBStep1;
struct StiMode3SequenceBStep2;
struct StiMode3SequenceBStep3;
struct StiMode3SequenceBStep4;
struct StiMode3SequenceBStep5;
struct StiMode3SequenceBStep6;
struct StiMode3SequenceBStep7;
struct StiMode3SequenceBStep8;
struct StiMode3SequenceBStep9;
}  // namespace mode_3_sequence_b

struct SsMode4SequenceB;

namespace mode_4_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiMode4SequenceB;
struct StiMode4SequenceBStep1;
struct StiMode4SequenceBStep2;
struct StiMode4SequenceBStep3;
struct StiMode4SequenceBStep4;
struct StiMode4SequenceBStep5;
struct StiMode4SequenceBStep6;
struct StiMode4SequenceBStep7;
struct StiMode4SequenceBStep8;
struct StiMode4SequenceBStep9;
}  // namespace mode_4_sequence_b

struct SsMode5SequenceB;

namespace mode_5_sequence_b
{
//FORWARD DECLARATIONS OF ALL INNER STATES
struct StiMode5SequenceBLoop;
struct StiMode5SequenceBStep1;
struct StiMode5SequenceBStep2;
struct StiMode5SequenceBStep3;
struct StiMode5SequenceBStep4;
struct StiMode5SequenceBStep5;
struct StiMode5SequenceBStep6;
struct StiMode5SequenceBStep7;
struct StiMode5SequenceBStep8;
struct StiMode5SequenceBStep9;
}  // namespace mode_5_sequence_b

//STATES
struct Mode1StObserve;
struct StRecoveryAnalyze1;
struct StRecoveryBifurcate1;
struct StRecoveryCalculate1;
struct StRecoveryDeliberate1;
struct StRecoveryEvaluate1;
struct StRecoveryGenerate1;
struct StRecoveryInnervate1;

struct Mode2StObserve;
struct StRecoveryAnalyze2;
struct StRecoveryBifurcate2;
struct StRecoveryCalculate2;
struct StRecoveryDeliberate2;
struct StRecoveryEvaluate2;
struct StRecoveryGenerate2;
struct StRecoveryInnervate2;

struct Mode3StObserve;
struct Mode4StObserve;
struct Mode5StObserve;


struct Mode1SequenceALoop;
struct Mode1SequenceBLoop;
struct Mode2SequenceALoop;
struct Mode2SequenceBLoop;
struct Mode3SequenceALoop;
struct Mode3SequenceBLoop;
struct Mode4SequenceALoop;
struct Mode4SequenceCLoop;
struct Mode4SequenceDLoop;
class Mode4SequenceBLoop;
struct Mode5SequenceALoop;
struct Mode5SequenceBLoop;


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
    this->createOrthogonal<OrKeyboard>();
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

#include <sm_multi_stage_1/states/mode_1_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a_loop.hpp>

#include <sm_multi_stage_1/states/mode_4_sequence_c_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d_loop.hpp>

#include <sm_multi_stage_1/states/mode_1_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b_loop.hpp>

#include <sm_multi_stage_1/states/mode_1_st_observe.hpp>
#include <sm_multi_stage_1/states/mode_2_st_observe.hpp>
#include <sm_multi_stage_1/states/mode_3_st_observe.hpp>
#include <sm_multi_stage_1/states/mode_4_st_observe.hpp>
#include <sm_multi_stage_1/states/mode_5_st_observe.hpp>

#include <sm_multi_stage_1/superstates/ss_mode_1_sequence_a.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_2_sequence_a.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_3_sequence_a.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_4_sequence_a.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_5_sequence_a.hpp>

#include <sm_multi_stage_1/superstates/ss_mode_4_sequence_c.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_4_sequence_d.hpp>

#include <sm_multi_stage_1/superstates/ss_mode_1_sequence_b.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_2_sequence_b.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_3_sequence_b.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_4_sequence_b.hpp>
#include <sm_multi_stage_1/superstates/ss_mode_5_sequence_b.hpp>

//ss_mode_1_sequence_a
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_4.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_3.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_1.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_2.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_5.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_9.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_8.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_7.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_a/sti_mode_1_sequence_a_step_6.hpp>

//ss_mode_2_sequence_a
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_4.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_3.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_1.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_2.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_5.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_9.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_8.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_7.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_a/sti_mode_2_sequence_a_step_6.hpp>

//ss_mode_3_sequence_a
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_4.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_3.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_1.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_2.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_5.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_9.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_8.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_7.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_a/sti_mode_3_sequence_a_step_6.hpp>

//ss_mode_4_sequence_a
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_4.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_3.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_1.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_2.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_5.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_9.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_8.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_7.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_a/sti_mode_4_sequence_a_step_6.hpp>

//ss_mode_4_sequence_c
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_1.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_2.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_3.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_4.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_5.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_6.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_7.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_8.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_c/sti_mode_4_sequence_c_step_9.hpp>

//ss_mode_4_sequence_d
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_4.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_3.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_1.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_2.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_5.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_9.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_8.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_7.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_d/sti_mode_4_sequence_d_step_6.hpp>

//ss_mode_5_sequence_a
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_4.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_3.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_1.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_loop.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_2.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_5.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_9.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_8.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_7.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_a/sti_mode_5_sequence_a_step_6.hpp>

//ss_mode_1_sequence_b
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_4.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_3.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_1.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_2.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_5.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_9.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_8.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_7.hpp>
#include <sm_multi_stage_1/states/mode_1_sequence_b/sti_mode_1_sequence_b_step_6.hpp>

//ss_mode_2_sequence_b
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_4.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_3.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_1.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_2.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_5.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_9.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_8.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_7.hpp>
#include <sm_multi_stage_1/states/mode_2_sequence_b/sti_mode_2_sequence_b_step_6.hpp>

//ss_mode_3_sequence_b
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_4.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_3.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_1.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_2.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_5.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_9.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_8.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_7.hpp>
#include <sm_multi_stage_1/states/mode_3_sequence_b/sti_mode_3_sequence_b_step_6.hpp>

//ss_mode_4_sequence_b
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_4.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_3.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_1.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_2.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_5.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_9.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_8.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_7.hpp>
#include <sm_multi_stage_1/states/mode_4_sequence_b/sti_mode_4_sequence_b_step_6.hpp>

//ss_mode_5_seqence_b
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_4.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_3.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_1.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_loop.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_2.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_5.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_9.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_8.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_7.hpp>
#include <sm_multi_stage_1/states/mode_5_sequence_b/sti_mode_5_sequence_b_step_6.hpp>
