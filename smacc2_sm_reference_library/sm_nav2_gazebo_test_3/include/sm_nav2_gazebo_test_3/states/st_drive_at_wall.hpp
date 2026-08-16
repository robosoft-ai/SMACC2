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
// Collision-abort demo, step 2: command a drive straight AT the south wall
// (3.5 m requested, wall ~2 m away and spanning the whole arena). The behavior
// server's collision checking must abort the goal before impact - the EXPECTED
// outcome here is EvCbFailure, proving the genuine server-abort path end to
// end. Success would mean the collision checking failed to intervene.
struct StDriveAtWall : smacc2::SmaccState<StDriveAtWall, SmNav2GazeboTest3>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};
  struct COLLISION_ABORT_OK : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    // expected: collision checking aborts the goal -> failure event
    Transition<EvCbFailure<CbDriveOnHeading, OrNavigation>, StBackUpSafe, COLLISION_ABORT_OK>,
    // unexpected but non-fatal: server let the full distance through
    Transition<EvCbSuccess<CbDriveOnHeading, OrNavigation>, StBackUpSafe, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StBackUpSafe, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // slow approach so the abort is clearly visible
    configure_orthogonal<OrNavigation, CbDriveOnHeading>(3.5f, 0.1f);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "StDriveAtWall: onEntry() - driving at the south wall, expecting a collision abort");
  }
};

}  // namespace sm_nav2_gazebo_test_3
