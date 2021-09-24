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
struct StNavigateForward2 : smacc2::SmaccState<StNavigateForward2, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateDegrees5>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateToWaypointsX>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
}  // namespace sm_dance_bot
