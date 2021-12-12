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

#include <nav2z_client/client_behaviors/cb_navigate_forward.hpp>
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
using ::cl_nav2z::odom_tracker::OdomTracker;
using ::cl_nav2z::odom_tracker::WorkingMode;

using ::cl_nav2z::Pose;

CbNavigateForward::CbNavigateForward(float distance_meters) : forwardDistance(distance_meters) {}

CbNavigateForward::CbNavigateForward() {}

CbNavigateForward::~CbNavigateForward() {}

void CbNavigateForward::setForwardDistance(float distance_meters)
{
  if (distance_meters < 0)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] negative forward distance: " << distance_meters
                       << ". Resetting to 0.");
    distance_meters = 0;
  }
  this->forwardDistance = distance_meters;

  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] setting fw motion distance: " << this->forwardDistance);
}

void CbNavigateForward::onEntry()
{
  setForwardDistance(forwardDistance);

  // straight motion distance
  double dist = forwardDistance;

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Straight motion distance: " << dist);

  // get current pose
  auto p = moveBaseClient_->getComponent<Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();
  tf2::Transform currentPose;
  tf2::fromMsg(currentPoseMsg, currentPose);

  RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateForward] current pose: " << currentPoseMsg);

  // force global orientation if it is requested
  if (this->forceInitialOrientation)
  {
    currentPoseMsg.orientation = *forceInitialOrientation;
    RCLCPP_WARN_STREAM(
      getLogger(), "[CbNavigateForward] Forcing initial straight motion orientation: "
                     << currentPoseMsg.orientation);
  }

  // compute forward goal pose
  tf2::Transform forwardDeltaTransform;
  forwardDeltaTransform.setIdentity();
  forwardDeltaTransform.setOrigin(tf2::Vector3(dist, 0, 0));

  tf2::Transform targetPose = currentPose * forwardDeltaTransform;

  ClNav2Z::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();
  tf2::toMsg(targetPose, goal.pose.pose);
  RCLCPP_INFO_STREAM(getLogger(), "[CbNavigateForward] TARGET POSE FORWARD: " << goal.pose.pose);

  geometry_msgs::msg::PoseStamped currentStampedPoseMsg;
  currentStampedPoseMsg.header.frame_id = referenceFrame;
  currentStampedPoseMsg.header.stamp = getNode()->now();
  tf2::toMsg(currentPose, currentStampedPoseMsg.pose);

  odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
  if (odomTracker_ != nullptr)
  {
    auto pathname = this->getCurrentState()->getName() + " - "+ getName();
    odomTracker_->pushPath(pathname);
    odomTracker_->setStartPoint(currentStampedPoseMsg);
    odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);
  }

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setForwardPlanner();

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("forward_goal_checker");

  moveBaseClient_->sendGoal(goal);
}

void CbNavigateForward::onExit()
{
  if (odomTracker_)
  {
    this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
  }
}

}  // namespace cl_nav2z
