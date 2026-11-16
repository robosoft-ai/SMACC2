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
// The waypoint loop: every entry sends the next waypoint. The waypoint navigator posts
// EvWaypointN when waypoint N is reached; alpha and bravo go to the rotation state, which comes
// back here, and charlie leaves the loop.
struct StNavigateWaypoints : smacc2::SmaccState<StNavigateWaypoints, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvWaypoint0<ClNav2Z, OrNavigation>, StRotateAtWaypoint, SUCCESS>,
    Transition<EvWaypoint1<ClNav2Z, OrNavigation>, StRotateAtWaypoint, SUCCESS>,
    Transition<EvWaypoint2<ClNav2Z, OrNavigation>, StNavigateNamedWaypoint, SUCCESS>,
    Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StMissionFailed, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateNamedWaypoint, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};

}  // namespace sm_nav2_gazebo_test_4
