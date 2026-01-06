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
#include <cl_nav2z/common.hpp>

#include <cl_nav2z/client_behaviors/cb_navigate_global_position.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
using namespace ::cl_nav2z::odom_tracker;
using namespace smacc2;

CbNavigateGlobalPosition::CbNavigateGlobalPosition() {}

CbNavigateGlobalPosition::CbNavigateGlobalPosition(
  float x, float y, float yaw, std::optional<CbNavigateGlobalPositionOptions> options)
{
  auto p = geometry_msgs::msg::Point();
  p.x = x;
  p.y = y;
  goalPosition = p;
  goalYaw = yaw;

  if (options) this->options = *options;
}

void CbNavigateGlobalPosition::setGoal(const geometry_msgs::msg::Pose & pose)
{
  goalPosition = pose.position;
  goalYaw = tf2::getYaw(pose.orientation);
}

void CbNavigateGlobalPosition::onEntry()
{
  RCLCPP_INFO(getLogger(), "Entering Navigate Global position");
  RCLCPP_INFO(getLogger(), "Component requirements completed");

  cl_nav2z::CpPose * cpPose;
  this->requiresComponent(cpPose, ComponentRequirement::HARD);
  auto pose = cpPose->toPoseMsg();

  CpOdomTracker * odomTracker;
  this->requiresComponent(odomTracker, ComponentRequirement::HARD);

  CpPlannerSwitcher * plannerSwitcher;
  this->requiresComponent(plannerSwitcher, ComponentRequirement::HARD);

  plannerSwitcher->setDefaultPlanners(false);

  if (options.controllerName_)
  {
    plannerSwitcher->setDesiredController(*(options.controllerName_));
  }

  plannerSwitcher->commitPublish();

  CpGoalCheckerSwitcher * goalCheckerSwitcher;
  this->requiresComponent(goalCheckerSwitcher, ComponentRequirement::HARD);
  goalCheckerSwitcher->setGoalCheckerId("goal_checker");

  auto pathname = this->getCurrentState()->getName() + " - " + getName();
  odomTracker->pushPath(pathname);
  odomTracker->setStartPoint(pose);
  odomTracker->setWorkingMode(WorkingMode::RECORD_PATH);

  execute();
}

// auxiliary function that defines the motion that is requested to the nav2 action server
void CbNavigateGlobalPosition::execute()
{
  cl_nav2z::CpPose * p;
  this->requiresComponent(p, ComponentRequirement::HARD);

  auto referenceFrame = p->getReferenceFrame();
  // auto currentPoseMsg = p->toPoseMsg();

  RCLCPP_INFO(getLogger(), "Sending Goal to MoveBase");
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  //goal.pose.header.stamp = getNode()->now();

  goal.pose.pose.position = goalPosition;
  tf2::Quaternion q;
  q.setRPY(0, 0, goalYaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  this->sendGoal(goal);
}

// This is the substate destructor. This code will be executed when the
// workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
void CbNavigateGlobalPosition::onExit()
{
  RCLCPP_INFO(getLogger(), "Exiting move goal Action Client");
}

}  // namespace cl_nav2z
