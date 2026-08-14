// Copyright 2026 RobosoftAI Inc.
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

namespace sm_nav2_gazebo_test_2
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
//
// Moves the robot to the radial pattern center: free space in all four diagonal
// ray directions of the tb3_sandbox map.
struct StNavigateToWaypoint1 : smacc2::SmaccState<StNavigateToWaypoint1, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  // Behavior events (EvCbSuccess/EvCbFailure) are state-scoped and cannot go
  // stale: machine-scoped EvActionSucceeded events posted by the previous
  // state's navigation can survive the transition and fire spuriously here.
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS1::SsRadialPattern1, SUCCESS>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StFinalState, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Navigate to the radial pattern center: x=0.5, y=0.0, yaw=0.0
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.5, 0.0, 0.0);

    // Keyboard behavior for manual control
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StNavigateToWaypoint1: runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "StNavigateToWaypoint1: onEntry() - Navigating to radial pattern center (x=0.5, y=0.0)");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StNavigateToWaypoint1: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_2
