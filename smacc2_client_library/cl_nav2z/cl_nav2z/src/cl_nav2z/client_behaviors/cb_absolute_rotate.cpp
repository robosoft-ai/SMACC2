// Copyright 2025 Robosoft Inc.
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
#include <cl_nav2z/client_behaviors/cb_absolute_rotate.hpp>
#include <cl_nav2z/common.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
using namespace smacc2;

CbAbsoluteRotate::CbAbsoluteRotate() {}
CbAbsoluteRotate::CbAbsoluteRotate(float absoluteGoalAngleDegree)
{
  this->absoluteGoalAngleDegree = absoluteGoalAngleDegree;
}

void CbAbsoluteRotate::onEntry()
{
  listener = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
  double goal_angle = this->absoluteGoalAngleDegree;

  RCLCPP_INFO_STREAM(getLogger(), "[CbAbsoluteRotate] Absolute yaw Angle:" << goal_angle);

  CpPlannerSwitcher * plannerSwitcher;
  this->requiresComponent(plannerSwitcher, ComponentRequirement::SOFT);

  if (plannerSwitcher == nullptr)
  {
    RCLCPP_WARN(
      getLogger(), "[%s] no CpPlannerSwitcher; planners left as they are", getName().c_str());
  }
  else if (spinningPlanner && *spinningPlanner == SpinningPlanner::PureSpinning)
  {
    plannerSwitcher->setPureSpinningPlanner();
  }
  else
  {
    plannerSwitcher->setDefaultPlanners();
  }

  cl_nav2z::CpPose * p;
  this->requiresComponent(p, ComponentRequirement::HARD);

  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = referenceFrame;

  auto targetAngle = goal_angle * M_PI / 180.0;
  goal.pose.pose.position = currentPoseMsg.position;
  tf2::Quaternion q;
  q.setRPY(0, 0, targetAngle);
  goal.pose.pose.orientation = tf2::toMsg(q);

  cl_nav2z::odom_tracker::CpOdomTracker * odomTracker_;
  this->requiresComponent(odomTracker_, ComponentRequirement::SOFT);

  if (odomTracker_ != nullptr)
  {
    auto pathname = this->getCurrentState()->getName() + " - " + getName();
    odomTracker_->pushPath(pathname);
    odomTracker_->setStartPoint(p->toPoseStampedMsg());
    odomTracker_->setCurrentMotionGoal(goal.pose);
    odomTracker_->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);
  }

  CpGoalCheckerSwitcher * goalCheckerSwitcher;
  this->requiresComponent(goalCheckerSwitcher, ComponentRequirement::HARD);

  goalCheckerSwitcher->setGoalCheckerId("absolute_rotate_goal_checker");

  RCLCPP_INFO_STREAM(
    getLogger(),
    "[" << getName() << "] current pose yaw: " << tf2::getYaw(currentPoseMsg.orientation));
  RCLCPP_INFO_STREAM(
    getLogger(),
    "[" << getName() << "] goal pose yaw: " << tf2::getYaw(goal.pose.pose.orientation));
  this->sendGoal(goal);
}

}  // namespace cl_nav2z
