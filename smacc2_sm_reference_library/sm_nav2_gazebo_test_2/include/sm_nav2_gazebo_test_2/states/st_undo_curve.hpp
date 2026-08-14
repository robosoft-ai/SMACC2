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
// Retraces the curved path recorded during StNavigateWithCurve exactly
// backwards, ending at the curve start near the spawn point. This exercises
// CbUndoPathBackwards on a curved trajectory, unlike the straight rays of the
// radial pattern.
struct StUndoCurve : smacc2::SmaccState<StUndoCurve, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StNavigateToWaypoint1, SUCCESS>,
    // On failure the mission still proceeds to the radial pattern; the path
    // stack is cleared on exit either way
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StNavigateToWaypoint1, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint1, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // See config/nav2_params.yaml for the plugin instances referenced here
    cl_nav2z::CbUndoPathBackwardsOptions options;
    options.undoControllerName_ = "UndoBackwardLocalPlanner";
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>(options);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StUndoCurve: onEntry() - Undoing the curved path backwards");
  }

  void onExit()
  {
    // Drop whatever remains of the curve recording: the radial pattern phase
    // starts with a clean odom tracker
    ClNav2Z * navClient;
    this->requiresClient(navClient);

    auto odomTracker = navClient->getComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    odomTracker->clearPath();

    RCLCPP_INFO(getLogger(), "StUndoCurve: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_2
