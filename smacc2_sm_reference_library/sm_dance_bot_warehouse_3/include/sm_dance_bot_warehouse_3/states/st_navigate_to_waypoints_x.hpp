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

namespace sm_dance_bot_warehouse_3
{
// STATE DECLARATION
struct StNavigateToWaypointsX : smacc2::SmaccState<StNavigateToWaypointsX, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct TRANSITION_1 : SUCCESS
  {
  };
  struct TRANSITION_2 : SUCCESS
  {
  };
  struct TRANSITION_3 : SUCCESS
  {
  };
  struct TRANSITION_4 : SUCCESS
  {
  };
  struct TRANSITION_5 : SUCCESS
  {
  };
  struct TRANSITION_6 : SUCCESS
  {
  };
  struct TRANSITION_7 : SUCCESS
  {
  };
  struct TRANSITION_8 : SUCCESS
  {
  };
  struct TRANSITION_9 : SUCCESS
  {
  };
  struct TRANSITION_10 : SUCCESS
  {
  };
  struct TRANSITION_11 : SUCCESS
  {
  };
  struct TRANSITION_12 : SUCCESS
  {
  };
  struct TRANSITION_13 : SUCCESS
  {
  };
  struct TRANSITION_14 : SUCCESS
  {
  };
  struct TRANSITION_15 : SUCCESS
  {
  };
  struct TRANSITION_16 : SUCCESS
  {
  };
  struct TRANSITION_17 : SUCCESS
  {
  };
  struct TRANSITION_18 : SUCCESS
  {
  };
  struct TRANSITION_19 : SUCCESS
  {
  };
  struct TRANSITION_20 : SUCCESS
  {
  };
  struct TRANSITION_21 : SUCCESS
  {
  };
  struct TRANSITION_22 : SUCCESS
  {
  };
  struct TRANSITION_23 : SUCCESS
  {
  };
  struct TRANSITION_24 : SUCCESS
  {
  };
  struct TRANSITION_25 : SUCCESS
  {
  };
  struct TRANSITION_26 : SUCCESS
  {
  };
  struct TRANSITION_27 : SUCCESS
  {
  };
  struct TRANSITION_28 : SUCCESS
  {
  };
  struct TRANSITION_29 : SUCCESS
  {
  };
  struct TRANSITION_30 : SUCCESS
  {
  };

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvWaypoint0<ClNav2Z, OrNavigation>, StRotateDegrees1, TRANSITION_1>,
    Transition<EvWaypoint1<ClNav2Z, OrNavigation>, StRotateDegrees2, TRANSITION_2>,
    Transition<EvWaypoint2<ClNav2Z, OrNavigation>, StRotateDegrees1, TRANSITION_3>,
    Transition<EvWaypoint3<ClNav2Z, OrNavigation>, StRotateDegrees2, TRANSITION_4>,
    Transition<EvWaypoint4<ClNav2Z, OrNavigation>, StRotateDegrees1, TRANSITION_5>,
    Transition<EvWaypoint5<ClNav2Z, OrNavigation>, StRotateDegrees1, TRANSITION_6>,
    Transition<EvWaypoint6<ClNav2Z, OrNavigation>, StRotateDegrees2, TRANSITION_7>,
    Transition<EvWaypoint7<ClNav2Z, OrNavigation>, StRotateDegrees1, TRANSITION_8>,
    Transition<EvWaypoint8<ClNav2Z, OrNavigation>, StRotateDegrees2, TRANSITION_9>,
    Transition<EvWaypoint9<ClNav2Z, OrNavigation>, StNavigateToWaypointsX2, TRANSITION_10>,
    Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateToWaypointsX>
    >
    reactions;

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
}  // namespace sm_dance_bot_warehouse_3
