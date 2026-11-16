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

namespace sm_nav2_gazebo_test_4
{
// CbNavigateNextWaypointUntilReached: one waypoint per entry, re-entering on success, until the
// waypoint named delta is reached (bravo, charlie, delta).
struct StNavigateUntilReached : smacc2::SmaccState<StNavigateUntilReached, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvGoalWaypointReached<CbNavigateNextWaypointUntilReached, OrNavigation>, StForwardCutShort, SUCCESS>,
    Transition<EvCbSuccess<CbNavigateNextWaypointUntilReached, OrNavigation>, StNavigateUntilReached, CONTINUELOOP>,
    Transition<EvCbFailure<CbNavigateNextWaypointUntilReached, OrNavigation>, StMissionFailed, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StForwardCutShort, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateNextWaypointUntilReached>("delta");
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};

}  // namespace sm_nav2_gazebo_test_4
