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
#include <move_base_z_client_plugin/common.hpp>

#include <move_base_z_client_plugin/client_behaviors/cb_navigate_global_position.hpp>
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.hpp>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.hpp>
#include <move_base_z_client_plugin/components/pose/cp_pose.hpp>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

CbNavigateGlobalPosition::CbNavigateGlobalPosition() {}

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

  auto pose = moveBaseClient_->getComponent<cl_move_base_z::Pose>()->toPoseMsg();
  auto * odomTracker = moveBaseClient_->getComponent<OdomTracker>();

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setDefaultPlanners();

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("goal_checker");

  odomTracker->pushPath();
  odomTracker->setStartPoint(pose);
  odomTracker->setWorkingMode(WorkingMode::RECORD_PATH);

  execute();
}

// auxiliary function that defines the motion that is requested to the move_base action server
void CbNavigateGlobalPosition::execute()
{
  auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  // auto currentPoseMsg = p->toPoseMsg();

  RCLCPP_INFO(getLogger(), "Sending Goal to MoveBase");
  ClMoveBaseZ::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();
  readStartPoseFromParameterServer(goal);

  // store the start pose on the state machine storage so that it can
  // be referenced from other states (for example return to radial start)
  this->getStateMachine()->setGlobalSMData("radial_start_pose", goal.pose);

  moveBaseClient_->sendGoal(goal);
}

void CbNavigateGlobalPosition::readStartPoseFromParameterServer(ClMoveBaseZ::Goal & goal)
{
  if (!goalPosition)
  {
    this->getCurrentState()->getParam("start_position_x", goal.pose.pose.position.x);
    this->getCurrentState()->getParam("start_position_y", goal.pose.pose.position.y);
    double yaw;
    this->getCurrentState()->getParam("start_position_yaw", yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal.pose.pose.orientation = tf2::toMsg(q);
  }
  else
  {
    goal.pose.pose.position = *goalPosition;
    tf2::Quaternion q;
    q.setRPY(0, 0, *goalYaw);
    goal.pose.pose.orientation = tf2::toMsg(q);
  }

  RCLCPP_INFO_STREAM(
    getLogger(), "start position read from parameter server: " << goal.pose.pose.position);
}

// This is the substate destructor. This code will be executed when the
// workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
void CbNavigateGlobalPosition::onExit()
{
  RCLCPP_INFO(getLogger(), "Exiting move goal Action Client");
}

}  // namespace cl_move_base_z
