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
// Second leg of the chained-undo phase: from P1 to P2 (-3.5, -0.5) on the west
// side of the arena, curving around the first pillar's west flank in open space.
// Leg 1's trail is pushed onto the odom tracker stack; after StUndoChain2
// retraces this leg, popPath restores leg 1 for StUndoChain1.
struct StNavigateChain2 : smacc2::SmaccState<StNavigateChain2, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StUndoChain2, SUCCESS>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StUndoChain2, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Chain leg 2: west side of the arena, curving around the first pillar
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-3.5, -0.5, M_PI);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StNavigateChain2: onEntry() - chained-undo phase, leg 2 to P2 (-3.5, -0.5)");
  }
};

}  // namespace sm_nav2_gazebo_test_2
