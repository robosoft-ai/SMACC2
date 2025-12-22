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

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct StNavigateToWaypoint2 : smacc2::SmaccState<StNavigateToWaypoint2, SmNav2GazeboTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<smacc2::EvActionSucceeded<ClNav2Z, OrNavigation>, StFinalState, SUCCESS>,
    Transition<smacc2::EvActionAborted<ClNav2Z, OrNavigation>, StFinalState, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Navigate to waypoint 2: back near origin (x=0.0, y=0.0, yaw=0.0)
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.0, 0.0, 0.0);

    // Keyboard behavior for manual control
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StNavigateToWaypoint2: runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StNavigateToWaypoint2: onEntry() - Navigating to waypoint 2 (x=0.0, y=0.0)");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StNavigateToWaypoint2: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_1
