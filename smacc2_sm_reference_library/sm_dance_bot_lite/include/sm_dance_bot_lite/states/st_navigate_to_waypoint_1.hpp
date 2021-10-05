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
namespace sm_dance_bot_lite
{
// STATE DECLARATION
struct StNavigateToWaypoint1 : smacc2::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StNavigateToWaypointsX>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToWaypointsX>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0, 0, 0);
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrStringPublisher, CbStringPublisher>("All Done!");
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
}  // namespace sm_dance_bot_lite
