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

#include <cl_keyboard/client_behaviors/cb_keyboard_twist_teleop.hpp>
#include <smacc2/smacc.hpp>

namespace sm_nav2_gazebo_test_3
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
//
// Collision-guard demo, final act: assisted teleop in front of the south wall.
// The robot arrives ~1.2 m from the wall, facing it, and waits for input. With
// the keyboard server terminal focused, the arrow keys drive the robot through
// the behavior server's assisted_teleop filter, which projects every command
// through the costmap and clamps it before a collision - prevention, where
// StDriveAtWall showed abort (try to ram the wall). Letters keep working from
// the same terminal; N ends the session, cancelling the action through the
// behavior base.
struct StAssistedTeleopGuard : smacc2::SmaccState<StAssistedTeleopGuard, SmNav2GazeboTest3>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct GUARD_DONE : SUCCESS {};
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbAssistedTeleop, OrNavigation>, StFinalState, GUARD_DONE>,
    Transition<EvCbFailure<CbAssistedTeleop, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StFinalState, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // zero allowance = unlimited window (nav2 only arms the timeout when
    // allowance > 0): teleop runs until N ends the state, which cancels the
    // action through the behavior base
    configure_orthogonal<OrNavigation, CbAssistedTeleop>(std::chrono::seconds(0));

    // arrow-key driving only: zero idle twist, the robot moves when driven
    configure_orthogonal<OrKeyboard, CbKeyboardTwistTeleop>(0.15f, 0.4f);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "StAssistedTeleopGuard: onEntry() - assisted teleop at the wall (idle push forward; "
      "arrow keys drive, N ends the state)");
  }
};

}  // namespace sm_nav2_gazebo_test_3
