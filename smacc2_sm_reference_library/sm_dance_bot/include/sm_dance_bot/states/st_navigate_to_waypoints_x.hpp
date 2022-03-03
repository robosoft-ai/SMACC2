// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc2/smacc.hpp>

namespace sm_dance_bot
{
// STATE DECLARATION
struct StNavigateToWaypointsX : smacc2::SmaccState<StNavigateToWaypointsX, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct TRANSITION_1 : SUCCESS{};
  struct TRANSITION_2 : SUCCESS{};
  struct TRANSITION_3 : SUCCESS{};
  struct TRANSITION_4 : SUCCESS{};
  struct TRANSITION_5 : SUCCESS{};
  struct TRANSITION_6 : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvWaypoint0<ClNav2Z, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_1>,
    Transition<EvWaypoint1<ClNav2Z, OrNavigation>, SS2::SsRadialPattern2, TRANSITION_2>,
    Transition<EvWaypoint2<ClNav2Z, OrNavigation>, SS3::SsRadialPattern3, TRANSITION_3>,
    Transition<EvWaypoint3<ClNav2Z, OrNavigation>, SS4::SsFPattern1, TRANSITION_4>,
    Transition<EvWaypoint4<ClNav2Z, OrNavigation>, StNavigateToWaypointsX, TRANSITION_5>,
    Transition<EvWaypoint5<ClNav2Z, OrNavigation>, SS5::SsSPattern1, TRANSITION_6>,
    Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateToWaypointsX, ABORT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
    configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();

  }

  void runtimeConfigure() {}

  void onExit(ABORT)
  {
  }
};
}  // namespace sm_dance_bot
