// Copyright 2024 RobosoftAI Inc.
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

namespace sm_nav2_gazebo_test_1
{

using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace cl_nav2z;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct StAllSensorsGo : smacc2::SmaccState<StAllSensorsGo, SmNav2GazeboTest1>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};
  struct NAV2_READY : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitNav2Nodes, OrNavigation>, StSetInitialPose, NAV2_READY>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StSetInitialPose, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSetInitialPose, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Wait for Nav2 nodes to be ready (PlannerServer, ControllerServer, BtNavigator)
    configure_orthogonal<OrNavigation, CbWaitNav2Nodes>();

    // Timer countdown: triggers EvTimer after 15 seconds (backup if Nav2 ready signal not received)
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(15s);

    // Keyboard behavior: allows manual state advancement with 'N' key
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StAllSensorsGo: runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StAllSensorsGo: onEntry() - Waiting for Nav2 nodes to be ready...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StAllSensorsGo: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_1
