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

namespace sm_dance_bot_warehouse_2
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
  struct TRANSITION_7 : SUCCESS{};
  struct TRANSITION_8 : SUCCESS{};
  struct TRANSITION_9 : SUCCESS{};
  struct TRANSITION_10 : SUCCESS{};
  struct TRANSITION_11 : SUCCESS{};
  struct TRANSITION_12 : SUCCESS{};
  struct TRANSITION_13 : SUCCESS{};
  struct TRANSITION_14 : SUCCESS{};


  // TRANSITION TABLE
  typedef mpl::list<

    // Transition<EvCbSuccess<CbNavigateNextWaypoint, OrNavigation>, StForwardAisle, TRANSITION_1>,
    // Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateToWaypointsX, ABORT>

      Transition<EvWaypoint1<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_1>,
      Transition<EvWaypoint3<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_2>,
      Transition<EvWaypoint5<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_3>,
      Transition<EvWaypoint7<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_5>,
      Transition<EvWaypoint9<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_6>,

      Transition<EvWaypoint11<ClNav2Z, OrNavigation>, StNavigateToWaypointsX, TRANSITION_9>,
      Transition<EvWaypoint12<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_9>,
      Transition<EvWaypoint13<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_10>,
      Transition<EvWaypoint14<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_11>,
      Transition<EvWaypoint15<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_12>,
      Transition<EvWaypoint16<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_13>,
      Transition<EvWaypoint17<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_13>,
      Transition<EvWaypoint18<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_13>,
      Transition<EvWaypoint19<ClNav2Z, OrNavigation>, StForwardAisle, TRANSITION_14>,

      //Transition<EvCbSuccess<CbNavigateNextWaypoint, OrNavigation>, StForwardAisle, TRANSITION_1>,
      Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateToWaypointsX, ABORT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
    configure_orthogonal<OrNavigation, CbResumeSlam>();
  }

  void runtimeConfigure() {}

  void onExit(ABORT)
  {

  }
};
}  // namespace sm_dance_bot_warehouse_2
