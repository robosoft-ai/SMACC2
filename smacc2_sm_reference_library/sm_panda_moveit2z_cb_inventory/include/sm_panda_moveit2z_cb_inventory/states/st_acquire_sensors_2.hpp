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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_behaviors/cb_wait_action_server.hpp>
#include <smacc2/client_behaviors/cb_wait_topic.hpp>
#include <smacc2/client_behaviors/cb_ros_launch_2.hpp>

namespace sm_panda_moveit2z_cb_inventory
{
using namespace std::chrono_literals;
using namespace smacc2::default_events;
using namespace cl_keyboard;

using smacc2::client_behaviors::CbSleepFor;
using smacc2::client_behaviors::CbWaitActionServer;
using smacc2::client_behaviors::CbWaitTopic;
// using cl_nav2z::CbWaitNav2Nodes;
using smacc2::client_behaviors::CbRosLaunch2;

// STATE DECLARATION
struct StAcquireSensors2
    : smacc2::SmaccState<StAcquireSensors2, SmPandaMoveit2zCbInventory> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT AND TRANSITION TAGS
  struct ON_SENSORS_AVAILABLE : SUCCESS {};
  struct SrAcquireSensors;
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvAllGo<SrAllEventsGo, SrAcquireSensors>, StMoveJoints, ON_SENSORS_AVAILABLE>,

      Transition<EvCbSuccess<CbSleepFor, OrArm>, StAcquireSensors, ABORT>,

      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StMoveJoints, NEXT>  
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() 
  {
    configure_orthogonal<OrArm, CbActiveStop>();
    configure_orthogonal<OrArm, CbWaitActionServer>(10s);
    //configure_orthogonal<OrAssigner, CbWaitNav2Nodes>();
    configure_orthogonal<OrArm, CbSleepFor>(12s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();

    // Create State Reactor
    auto srAllSensorsReady = static_createStateReactor<
        SrAllEventsGo,
        smacc2::state_reactors::EvAllGo<SrAllEventsGo, SrAcquireSensors>,
        mpl::list<
            EvCbSuccess<CbWaitActionServer, OrArm>//,
            // EvCbSuccess<CbWaitNav2Nodes, OrAssigner>
            // EvCbSuccess<CbSleepFor, OrNavigation>>
            >>();
  }
};
} // namespace sm_panda_moveit2z_cb_inventory
