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
// First undo of the chain: retraces leg 2 back to P1. On success
// CbUndoPathBackwards pops the odom tracker stack, restoring leg 1's trail as
// the active path for StUndoChain1.
//
// IMPORTANT: no clearPath() in onExit here - it would destroy the just-popped
// leg 1 trail that the next state needs. Only the FINAL undo of a chain clears.
struct StUndoChain2 : smacc2::SmaccState<StUndoChain2, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StUndoChain1, SUCCESS>,
    // On failure the chain is broken (no pop happened): skip the second undo
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StNavigateToWaypoint1, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StUndoChain1, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    cl_nav2z::CbUndoPathBackwardsOptions options;
    options.undoControllerName_ = "UndoBackwardLocalPlanner";
    // Tight goal checker: this undo's end pose is the START of the next undo's
    // retrace, so any handoff error becomes StUndoChain1's initial tracking error
    options.goalCheckerId_ = "undo_path_backwards_goal_checker_2";
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>(options);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StUndoChain2: onEntry() - undoing chain leg 2 (back to P1)");
  }
};

}  // namespace sm_nav2_gazebo_test_2
