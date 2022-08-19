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

#include <nav2z_client/client_behaviors/cb_navigate_named_waypoint.hpp>

namespace cl_nav2z
{
CbNavigateNamedWaypoint::CbNavigateNamedWaypoint(
  std::string waypointname, std::optional<NavigateNextWaypointOptions> options)
: CbNavigateNextWaypoint(options)
{
  waypointname_ = waypointname;
}

CbNavigateNamedWaypoint::~CbNavigateNamedWaypoint() {}

void CbNavigateNamedWaypoint::onEntry()
{
  // waypointsNavigator_ = nav2zClient_->getComponent<WaypointNavigator>();
  // waypointsNavigator_->sendNextGoal(options_);

  // auto waypointname = waypointsNavigator_->getCurrentWaypointName();

  // if(waypointname)
  // {
  // RCLCPP_INFO(
  //   getLogger(), "[CbNavigateNamedWaypoint] current iteration waypoints i: %ld with name '%s'",
  //   waypointsNavigator_->getCurrentWaypointIndex(), waypointname->c_str());  }
  // else
  // {
  // RCLCPP_INFO(
  //   getLogger(), "[CbNavigateNamedWaypoint] current iteration waypoints i: %ld",
  //   waypointsNavigator_->getCurrentWaypointIndex());  }

  auto waypointsNavigator_ = nav2zClient_->getComponent<WaypointNavigator>();
  waypointsNavigator_->seekName(waypointname_);
  CbNavigateNextWaypoint::onEntry();
}

void CbNavigateNamedWaypoint::onExit() { CbNavigateNextWaypoint::onExit(); }

}  // namespace cl_nav2z
