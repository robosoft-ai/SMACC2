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
#include <nav2z_client/common.hpp>

#include <nav2z_client/client_behaviors/cb_navigate_global_position.hpp>
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/planner_switcher/planner_switcher.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
using namespace ::cl_nav2z::odom_tracker;

CbNavigateGlobalPosition::CbNavigateGlobalPosition(float x, float y, float yaw)
{
  auto p = geometry_msgs::msg::Point();
  p.x = x;
  p.y = y;
  goalPosition = p;
  goalYaw = yaw;
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

  auto pose = moveBaseClient_->getComponent<cl_nav2z::Pose>()->toPoseMsg();
  auto * odomTracker = moveBaseClient_->getComponent<OdomTracker>();

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setDefaultPlanners();

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
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
  auto p = moveBaseClient_->getComponent<cl_nav2z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  // auto currentPoseMsg = p->toPoseMsg();

  RCLCPP_INFO(getLogger(), "Sending Goal to MoveBase");
  ClNav2Z::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();

  goal.pose.pose.position = goalPosition;
  tf2::Quaternion q;
  q.setRPY(0, 0, goalYaw);
  goal.pose.pose.orientation = tf2::toMsg(q);
  // store the start pose on the state machine storage so that it can
  // be referenced from other states (for example return to radial start)
  this->getStateMachine()->setGlobalSMData("radial_start_pose", goal.pose);

  moveBaseClient_->sendGoal(goal);
}

// This is the substate destructor. This code will be executed when the
// workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
void CbNavigateGlobalPosition::onExit()
{
  RCLCPP_INFO(getLogger(), "Exiting move goal Action Client");
}

}  // namespace cl_nav2z
