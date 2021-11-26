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

#include <smacc2/smacc.hpp>

// CLIENTS
//#include <ros_timer_client/cl_ros_timer.hpp>
#include <keyboard_client/cl_keyboard.hpp>

// ORTHOGONALS
//#include "orthogonals/or_timer.hpp"
//#include "orthogonals/or_updatable_publisher.hpp"
#include "orthogonals/or_keyboard.hpp"
#include "orthogonals/or_subscriber.hpp"

// using namespace cl_ros_timer;
// using namespace cl_ros_publisher;
using namespace cl_keyboard;

using namespace sm_pubsub_1::cl_subscriber;

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

#include "clients/cl_subscriber/client_behaviors/cb_my_subscriber_behavior.hpp"

//#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
//#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>

// STATE REACTORS
//#include <sr_all_events_go/sr_all_events_go.hpp>
#include <eg_conditional_generator/eg_conditional_generator.hpp>

using namespace smacc2;
using namespace smacc2::state_reactors;
using namespace smacc2::default_events;
using namespace smacc2::event_generators;

namespace sm_pubsub_1
{
// SUPERSTATES
namespace SS1
{
class Ss1;
}  // namespace SS1

// SUPERSTATES
namespace SS2
{
class Ss2;
}  // namespace SS2

// STATES
class StState1;  // first state specially needs a forward declaration
class StState2;
class StState3;
class StState4;

class MsRun;
class MsRecover;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmPubsub1 : public smacc2::SmaccStateMachineBase<SmPubsub1, MsRun>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    // this->createOrthogonal<OrTimer>();
    // this->createOrthogonal<OrUpdatablePublisher>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrSubscriber>();
  }
};
}  // namespace sm_pubsub_1

// MODE STATES
#include <sm_pubsub_1/mode_states/ms_run.hpp>

#include <sm_pubsub_1/mode_states/ms_recover.hpp>

// STATES
#include <sm_pubsub_1/states/st_state_1.hpp>
#include <sm_pubsub_1/states/st_state_2.hpp>
#include <sm_pubsub_1/states/st_state_3.hpp>
#include <sm_pubsub_1/states/st_state_4.hpp>
#include <sm_pubsub_1/superstates/ss_superstate_1.hpp>
#include <sm_pubsub_1/superstates/ss_superstate_2.hpp>
