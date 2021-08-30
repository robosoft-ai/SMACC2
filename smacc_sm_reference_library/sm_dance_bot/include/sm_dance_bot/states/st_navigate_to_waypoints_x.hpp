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

#include <smacc2/smacc.hpp>
namespace sm_dance_bot
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

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvWaypoint0<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_1>,
    Transition<EvWaypoint1<ClMoveBaseZ, OrNavigation>, SS2::SsRadialPattern2, TRANSITION_2>,
    Transition<EvWaypoint2<ClMoveBaseZ, OrNavigation>, SS3::SsRadialPattern3, TRANSITION_3>,
    Transition<EvWaypoint3<ClMoveBaseZ, OrNavigation>, SS4::SsFPattern1, TRANSITION_4>,
    Transition<EvWaypoint4<ClMoveBaseZ, OrNavigation>, SS5::SsSPattern1, TRANSITION_5>,
    Transition<EvCbFailure<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
  }

  void runtimeConfigure() {}
};
}  // namespace sm_dance_bot
