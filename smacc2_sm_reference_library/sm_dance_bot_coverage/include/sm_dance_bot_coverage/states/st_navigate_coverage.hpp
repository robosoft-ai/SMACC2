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
#include <sm_dance_bot_coverage/clients/cl_navigation_coverage/cl_navigation_coverage.hpp>
#include <sm_dance_bot_coverage/clients/cl_navigation_coverage/client_behaviors/cb_navigate_coverage.hpp>

namespace sm_dance_bot_coverage
{
// STATE DECLARATION
struct StNavigateCoverage : smacc2::SmaccState<StNavigateCoverage, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    // Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StNavigateToWaypointsX>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateCoverage>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateCoverage>();
  }

  void runtimeConfigure()
  {
    auto cb = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateCoverage>();
    geometry_msgs::msg::Polygon polygon;
    //field = [[5.0, 5.0], [5.0, 15.0], [15.0, 15.0], [10.0, 5.0], [5.0, 5.0]]

    polygon.points.resize(5);
    polygon.points[0].x = 5.0;
    polygon.points[0].y = 5.0;
    polygon.points[1].x = 5.0;
    polygon.points[1].y = 15.0;
    polygon.points[2].x = 15.0;
    polygon.points[2].y = 15.0;
    polygon.points[3].x = 10.0;
    polygon.points[3].y = 5.0;
    polygon.points[4].x = 5.0;
    polygon.points[4].y = 5.0;

    cb->perimeter_.push_back(polygon);
    cb->frame_id_ = "map";
  }
};
}  // namespace sm_dance_bot_coverage

