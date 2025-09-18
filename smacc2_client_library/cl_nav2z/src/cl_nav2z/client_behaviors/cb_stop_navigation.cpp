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

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/client_behaviors/cb_abort_navigation.hpp>
#include <cl_nav2z/common.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

#include <rclcpp/parameter_client.hpp>

namespace cl_nav2z
{
CbStopNavigation::CbStopNavigation() {}

void CbStopNavigation::onEntry()
{
  // this->sendGoal(goal);

  // this->cancelGoal();

  cl_nav2z::CpPose * poseComponent;
  this->requiresComponent(poseComponent, ComponentRequirement::HARD);

  this->setGoal(poseComponent->toPoseMsg());

  CbNavigateGlobalPosition::onEntry();

  // this->sendGoal(goal);
}

void CbStopNavigation::onExit() {}

}  // namespace cl_nav2z
