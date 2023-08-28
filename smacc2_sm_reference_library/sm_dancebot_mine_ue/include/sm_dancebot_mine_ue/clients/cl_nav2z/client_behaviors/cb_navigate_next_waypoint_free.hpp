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
#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator_base.hpp>
#include <sm_dancebot_mine_ue/clients/cl_nav2z/client_behaviors/cb_position_control_free_space.hpp>

namespace sm_dancebot_mine_ue
{

class CbNavigateNextWaypointFree : public sm_dancebot_mine_ue::CbPositionControlFreeSpace
{
public:
  CbNavigateNextWaypointFree() { }

  virtual ~CbNavigateNextWaypointFree() {}

  void onEntry() override 
  {
    requiresComponent(this->waypointsNavigator_); 
    this->target_pose_ = this->waypointsNavigator_->getCurrentPose();

    this->onSuccess(&CbNavigateNextWaypointFree::onSucessCallback, this);
    RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateNextWaypoint] initial load file target pose: x: " << this->target_pose_.position.x << ", y: " << this->target_pose_.position.y); 
    CbPositionControlFreeSpace::onEntry();
  }

  void onSucessCallback() 
  {
    RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateNextWaypoint] Success on planning to next waypoint"); 
    this->waypointsNavigator_->notifyGoalReached();
    this->waypointsNavigator_->forward(1);
    RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateNextWaypoint] next position index: " << this->waypointsNavigator_->getCurrentWaypointIndex() << "/" << this->waypointsNavigator_->getWaypoints().size());
  }

  void onExit() override 
  {
  }

protected:
  cl_nav2z::CpWaypointNavigatorBase * waypointsNavigator_;
};

}  // namespace sm_dancebot_mine_ue
