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

#include <nav2z_client/client_behaviors/cb_navigate_next_waypoint_until_reached.hpp>

namespace cl_nav2z
{
CbNavigateNextWaypointUntilReached::CbNavigateNextWaypointUntilReached(
    std::string goalWaypointName, std::optional<NavigateNextWaypointOptions> options)
  : CbNavigateNextWaypoint(options), goalWaypointName_(goalWaypointName)
{
}

CbNavigateNextWaypointUntilReached::~CbNavigateNextWaypointUntilReached()
{
}

void CbNavigateNextWaypointUntilReached::onEntry()
{
  waypointsNavigator_ = moveBaseClient_->getComponent<WaypointNavigator>();

  auto current_waypoint_name = waypointsNavigator_->getCurrentWaypointName();

  if (current_waypoint_name == this->goalWaypointName_)
  {
    RCLCPP_INFO(getLogger(),
                "[CbNavigateNextWaypointUntilReached] GoalReached current iteration waypoint i: %ld with name '%s'",
                waypointsNavigator_->getCurrentWaypointIndex(), current_waypoint_name->c_str());

    this->postEvGoalWaypointReached_();
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbNavigateNextWaypointUntilReached] goal:'%s' current:'%s'. keep navigating.",
                goalWaypointName_.c_str(), current_waypoint_name->c_str());
  }

  CbNavigateNextWaypoint::onEntry();
}

void CbNavigateNextWaypointUntilReached::onExit()
{
  CbNavigateNextWaypoint::onExit();
}

}  // namespace cl_nav2z
