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

// CLIENTS
#include <keyboard_client/cl_keyboard.h>
#include <ros_timer_client/cl_ros_timer.h>

// ORTHOGONALS
#include <sm_three_some/orthogonals/or_keyboard.h>
#include <sm_three_some/orthogonals/or_subscriber.h>
#include <sm_three_some/orthogonals/or_timer.h>
#include <sm_three_some/orthogonals/or_updatable_publisher.h>

using namespace cl_ros_timer;
using namespace cl_ros_publisher;
using namespace cl_keyboard;
using namespace sm_three_some::cl_subscriber;

// CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>
#include <sm_three_some/clients/cl_subscriber/client_behaviors/cb_default_subscriber_behavior.h>
#include <sm_three_some/clients/cl_subscriber/client_behaviors/cb_watchdog_subscriber_behavior.h>

//#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

// STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;

namespace sm_three_some
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
struct SmThreeSome : public smacc::SmaccStateMachineBase<SmThreeSome, MsRun>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrUpdatablePublisher>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrSubscriber>();
  }
};
}  // namespace sm_three_some

// MODE STATES
#include <sm_three_some/mode_states/ms_run.h>

#include <sm_three_some/mode_states/ms_recover.h>

// STATES
#include <sm_three_some/states/st_state_1.h>
#include <sm_three_some/states/st_state_2.h>
#include <sm_three_some/states/st_state_3.h>
#include <sm_three_some/states/st_state_4.h>
#include <sm_three_some/superstates/ss_superstate_1.h>
#include <sm_three_some/superstates/ss_superstate_2.h>
