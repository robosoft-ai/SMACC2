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

#include <nav2z_client/client_behaviors/cb_rotate.hpp>
#include <nav2z_client/common.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
CbRotate::CbRotate(float rotate_degree) : rotateDegree(rotate_degree) {}

CbRotate::CbRotate(float rotate_degree, SpinningPlanner spinning_planner)
: rotateDegree(rotate_degree), spinningPlanner(spinning_planner)
{
}

void CbRotate::onEntry()
{
  double angle_increment_degree = rotateDegree;

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();

  if (spinningPlanner && *spinningPlanner == SpinningPlanner::PureSpinning)
  {
    plannerSwitcher->setPureSpinningPlanner();
  }
  else
  {
    plannerSwitcher->setDefaultPlanners();
  }

  auto p = moveBaseClient_->getComponent<cl_nav2z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();

  tf2::Transform currentPose;
  tf2::fromMsg(currentPoseMsg, currentPose);

  auto odomTracker = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
  ClNav2Z::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();

  auto currentAngle = tf2::getYaw(currentPoseMsg.orientation);
  auto targetAngle = currentAngle + angle_increment_degree * M_PI / 180.0;
  goal.pose.pose.position = currentPoseMsg.position;
  tf2::Quaternion q;
  q.setRPY(0, 0, targetAngle);
  goal.pose.pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::PoseStamped stampedCurrentPoseMsg;
  stampedCurrentPoseMsg.header.frame_id = referenceFrame;
  stampedCurrentPoseMsg.header.stamp = getNode()->now();
  stampedCurrentPoseMsg.pose = currentPoseMsg;

  this->requiresClient(moveBaseClient_);
  auto pathname = this->getCurrentState()->getName() + " - " + getName();
  odomTracker->pushPath(pathname);

  odomTracker->setStartPoint(stampedCurrentPoseMsg);
  odomTracker->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);

  RCLCPP_INFO_STREAM(getLogger(), "current pose: " << currentPoseMsg);
  RCLCPP_INFO_STREAM(getLogger(), "goal pose: " << goal.pose.pose);
  moveBaseClient_->sendGoal(goal);
}

}  // namespace cl_nav2z
