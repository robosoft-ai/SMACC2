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

namespace sm_nav2_gazebo_test_3
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
//
// Collision-abort demo, step 1: quarter turn to face the south wall (~2 m away).
// The wall spans the whole arena, so accumulated dead-reckoning drift from the
// primitive laps cannot make the approach miss (a pillar target proved too
// fragile: ~13 degrees of subtended angle at 1.5 m).
struct StSpinToWall : smacc2::SmaccState<StSpinToWall, SmNav2GazeboTest3>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSpin, OrNavigation>, StDriveAtWall, SUCCESS>,
    Transition<EvCbFailure<CbSpin, OrNavigation>, StDriveAtWall, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StDriveAtWall, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // face the south wall: quarter turn clockwise from east
    configure_orthogonal<OrNavigation, CbSpin>(-M_PI / 2);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StSpinToWall: onEntry() - collision-abort demo, turning to face the pillar");
  }
};

}  // namespace sm_nav2_gazebo_test_3
