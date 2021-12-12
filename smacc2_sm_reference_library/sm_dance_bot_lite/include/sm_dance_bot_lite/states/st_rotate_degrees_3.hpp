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

#include <smacc2/smacc.hpp>
namespace sm_dance_bot_lite
{
// STATE DECLARATION
struct StRotateDegrees3 : smacc2::SmaccState<StRotateDegrees3, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbRotate, OrNavigation>, StNavigateForward1, SUCCESS>,
    Transition<EvCbFailure<CbRotate, OrNavigation>, StRotateDegrees3>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbRotate>(/*30*/ 720);
    //configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure()
  {
    auto cbrotate = this->getOrthogonal<OrNavigation>()
                    ->getClientBehavior<CbAbsoluteRotate>();

    cbrotate->spinningPlanner = cl_nav2z::SpinningPlanner::PureSpinning;

  }
};
}  // namespace sm_dance_bot_lite
