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

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav2z_client/client_behaviors/cb_navigate_backwards.hpp>
#include <nav2z_client/common.hpp>
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

namespace cl_nav2z
{
using namespace ::cl_nav2z::odom_tracker;

CbNavigateBackwards::CbNavigateBackwards(float backwardDistance)
{
  if (backwardDistance < 0)
  {
    RCLCPP_ERROR(getLogger(), "[CbNavigateBackwards] distance must be greater or equal than 0");
    this->backwardDistance = 0;
  }
  this->backwardDistance = backwardDistance;
}

void CbNavigateBackwards::onEntry()
{
  // straight motion distance
  double dist = backwardDistance;

  RCLCPP_INFO_STREAM(
    getLogger(), "[CbNavigateBackwards] Straight backwards motion distance: " << dist);

  auto p = moveBaseClient_->getComponent<cl_nav2z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();
  tf2::Transform currentPose;
  tf2::fromMsg(currentPoseMsg, currentPose);

  tf2::Transform backwardDeltaTransform;
  backwardDeltaTransform.setIdentity();
  backwardDeltaTransform.setOrigin(tf2::Vector3(-dist, 0, 0));

  tf2::Transform targetPose = currentPose * backwardDeltaTransform;

  ClNav2Z::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();
  tf2::toMsg(targetPose, goal.pose.pose);
  RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateBackwards] TARGET POSE BACKWARDS: " << goal.pose);

  geometry_msgs::msg::PoseStamped currentStampedPoseMsg;
  currentStampedPoseMsg.header.frame_id = referenceFrame;
  currentStampedPoseMsg.header.stamp = getNode()->now();
  tf2::toMsg(currentPose, currentStampedPoseMsg.pose);

  odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
  if (odomTracker_ != nullptr)
  {
    this->odomTracker_->clearPath();
    this->odomTracker_->setStartPoint(currentStampedPoseMsg);
    this->odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);
  }

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setBackwardPlanner();

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("backward_goal_checker");

  moveBaseClient_->sendGoal(goal);
}

void CbNavigateBackwards::onExit()
{
  if (odomTracker_)
  {
    this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
  }
}

}  // namespace cl_nav2z
