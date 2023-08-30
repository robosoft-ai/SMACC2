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

namespace sm_dancebot_mine_ue
{
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;

// STATE DECLARATION
struct StTurnAround : smacc2::SmaccState<StTurnAround, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StNavigateReverseMineWaypointsX, SUCCESS>
    Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StNavigateReverseMineWaypointsX, SUCCESS>
    
    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrNavigation, CbPureSpinning>(M_PI, 0.8);
    configure_orthogonal<OrNavigation,CbSleepFor>(2s);
    configure_orthogonal<OrNavigation, sm_dancebot_mine_ue::CbLoadWaypointsFile>("waypoints_mine_reverse", "sm_dancebot_mine_ue");
  }

  void runtimeConfigure()
  {
    // auto spinningBehavior = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbPureSpinning>();
    // spinningBehavior->yaw_goal_tolerance_rads_ = 0.2;
    
  }
};
}  // namespace sm_dancebot_ue
