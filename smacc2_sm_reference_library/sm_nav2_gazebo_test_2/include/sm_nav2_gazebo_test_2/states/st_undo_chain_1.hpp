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
// Second undo of the chain: retraces the restored leg 1 trail back to the
// start. This is the final undo of the chain, so the remaining path is cleared
// on exit.
struct StUndoChain1 : smacc2::SmaccState<StUndoChain1, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StNavigateToWaypoint1, SUCCESS>,
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StNavigateToWaypoint1, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint1, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    cl_nav2z::CbUndoPathBackwardsOptions options;
    options.undoControllerName_ = "UndoBackwardLocalPlanner";
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>(options);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StUndoChain1: onEntry() - undoing chain leg 1 (back to start)");
  }

  void onExit()
  {
    // End of the chain: drop whatever remains of the recorded paths
    ClNav2Z * navClient;
    this->requiresClient(navClient);

    auto odomTracker = navClient->getComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    odomTracker->clearPath();

    RCLCPP_INFO(getLogger(), "StUndoChain1: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_2
