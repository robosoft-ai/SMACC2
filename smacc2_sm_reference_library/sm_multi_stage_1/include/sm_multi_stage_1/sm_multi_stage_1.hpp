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
class StiASequenceStep11;
class StiASequenceStep21;
class StiASequenceStep31;
class StiASequenceStep41;
class StiASequenceStep51;
class StiASequenceStep61;
class StiASequenceStep71;
class StiASequenceStep81;
class StiASequenceStep91;
}  // namespace a_sequence_1

class SsASequence2;
namespace a_sequence_2
{
class StiASequenceLoop2;
class StiASequenceStep12;
class StiASequenceStep22;
class StiASequenceStep32;
class StiASequenceStep42;
class StiASequenceStep52;
class StiASequenceStep62;
class StiASequenceStep72;
class StiASequenceStep82;
class StiASequenceStep92;
}  // namespace a_sequence_2

class SsASequence3;
namespace a_sequence_3
{
class StiASequenceLoop3;
class StiASequenceStep13;
class StiASequenceStep23;
class StiASequenceStep33;
class StiASequenceStep43;
class StiASequenceStep53;
class StiASequenceStep63;
class StiASequenceStep73;
class StiASequenceStep83;
class StiASequenceStep93;
}  // namespace a_sequence_3

class SsASequence4;
namespace a_sequence_4
{
class StiASequenceLoop4;
class StiASequenceStep14;
class StiASequenceStep24;
class StiASequenceStep34;
class StiASequenceStep44;
class StiASequenceStep54;
class StiASequenceStep64;
class StiASequenceStep74;
class StiASequenceStep84;
class StiASequenceStep94;
}  // namespace a_sequence_4

class SsCSequence4;
namespace c_sequence_4
{
class StiCSequenceLoop4;
class StiCSequenceStep14;
class StiCSequenceStep24;
class StiCSequenceStep34;
class StiCSequenceStep44;
class StiCSequenceStep54;
class StiCSequenceStep64;
class StiCSequenceStep74;
class StiCSequenceStep84;
class StiCSequenceStep94;
}  // namespace c_sequence_4

class SsDSequence4;
namespace d_sequence_4
{
class StiDSequenceLoop4;
class StiDSequenceStep14;
class StiDSequenceStep24;
class StiDSequenceStep34;
class StiDSequenceStep44;
class StiDSequenceStep54;
class StiDSequenceStep64;
class StiDSequenceStep74;
class StiDSequenceStep84;
class StiDSequenceStep94;
}  // namespace d_sequence_4

class SsASequence5;
namespace a_sequence_5
{
class StiASequenceLoop5;
class StiASequenceStep15;
class StiASequenceStep25;
class StiASequenceStep35;
class StiASequenceStep45;
class StiASequenceStep55;
class StiASequenceStep65;
class StiASequenceStep75;
class StiASequenceStep85;
class StiASequenceStep95;
}  // namespace a_sequence_5


class SsBSequence1;

namespace b_sequence_1
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop1;
class StiBSequenceStep11;
class StiBSequenceStep21;
class StiBSequenceStep31;
class StiBSequenceStep41;
class StiBSequenceStep51;
class StiBSequenceStep61;
class StiBSequenceStep71;
class StiBSequenceStep81;
class StiBSequenceStep91;
}  // namespace b_sequence_1

class SsBSequence2;

namespace b_sequence_2
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop2;
class StiBSequenceStep12;
class StiBSequenceStep22;
class StiBSequenceStep32;
class StiBSequenceStep42;
class StiBSequenceStep52;
class StiBSequenceStep62;
class StiBSequenceStep72;
class StiBSequenceStep82;
class StiBSequenceStep92;
}  // namespace b_sequence_2

class SsBSequence3;

namespace b_sequence_3
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop3;
class StiBSequenceStep13;
class StiBSequenceStep23;
class StiBSequenceStep33;
class StiBSequenceStep43;
class StiBSequenceStep53;
class StiBSequenceStep63;
class StiBSequenceStep73;
class StiBSequenceStep83;
class StiBSequenceStep93;
}  // namespace b_sequence_3

class SsBSequence4;

namespace b_sequence_4
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop4;
class StiBSequenceStep14;
class StiBSequenceStep24;
class StiBSequenceStep34;
class StiBSequenceStep44;
class StiBSequenceStep54;
class StiBSequenceStep64;
class StiBSequenceStep74;
class StiBSequenceStep84;
class StiBSequenceStep94;
}  // namespace b_sequence_4

class SsBSequence5;

namespace b_sequence_5
{
//FORWARD DECLARATIONS OF ALL INNER STATES
class StiBSequenceLoop5;
class StiBSequenceStep15;
class StiBSequenceStep25;
class StiBSequenceStep35;
class StiBSequenceStep45;
class StiBSequenceStep55;
class StiBSequenceStep65;
class StiBSequenceStep75;
class StiBSequenceStep85;
class StiBSequenceStep95;
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
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_4_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_3_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_1_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_loop_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_2_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_5_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_9_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_8_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_7_1.hpp>
#include <sm_multi_stage_1/states/a_sequence_1/sti_a_sequence_step_6_1.hpp>

//ss_a_sequence_2
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_4_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_3_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_1_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_loop_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_2_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_5_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_9_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_8_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_7_2.hpp>
#include <sm_multi_stage_1/states/a_sequence_2/sti_a_sequence_step_6_2.hpp>

//ss_a_sequence_3
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_4_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_3_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_1_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_loop_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_2_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_5_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_9_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_8_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_7_3.hpp>
#include <sm_multi_stage_1/states/a_sequence_3/sti_a_sequence_step_6_3.hpp>

//ss_a_sequence_4
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_4_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_3_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_1_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_2_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_5_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_9_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_8_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_7_4.hpp>
#include <sm_multi_stage_1/states/a_sequence_4/sti_a_sequence_step_6_4.hpp>

//ss_c_sequence_4
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_4_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_3_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_1_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_2_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_5_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_9_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_8_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_7_4.hpp>
#include <sm_multi_stage_1/states/c_sequence_4/sti_c_sequence_step_6_4.hpp>

//ss_d_sequence_4
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_4_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_3_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_1_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_2_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_5_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_9_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_8_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_7_4.hpp>
#include <sm_multi_stage_1/states/d_sequence_4/sti_d_sequence_step_6_4.hpp>

//ss_a_sequence_5
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_4_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_3_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_1_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_loop_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_2_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_5_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_9_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_8_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_7_5.hpp>
#include <sm_multi_stage_1/states/a_sequence_5/sti_a_sequence_step_6_5.hpp>

//ss_b_sequence_1
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_4_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_3_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_1_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_loop_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_2_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_5_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_9_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_8_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_7_1.hpp>
#include <sm_multi_stage_1/states/b_sequence_1/sti_b_sequence_step_6_1.hpp>

//ss_b_sequence_2
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_4_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_3_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_1_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_loop_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_2_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_5_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_9_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_8_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_7_2.hpp>
#include <sm_multi_stage_1/states/b_sequence_2/sti_b_sequence_step_6_2.hpp>

//ss_b_sequence_3
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_4_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_3_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_1_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_loop_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_2_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_5_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_9_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_8_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_7_3.hpp>
#include <sm_multi_stage_1/states/b_sequence_3/sti_b_sequence_step_6_3.hpp>

//ss_b_sequence_4
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_4_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_3_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_1_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_loop_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_2_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_5_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_9_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_8_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_7_4.hpp>
#include <sm_multi_stage_1/states/b_sequence_4/sti_b_sequence_step_6_4.hpp>

//ss_b_sequence_5
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_4_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_3_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_1_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_loop_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_2_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_5_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_9_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_8_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_7_5.hpp>
#include <sm_multi_stage_1/states/b_sequence_5/sti_b_sequence_step_6_5.hpp>
