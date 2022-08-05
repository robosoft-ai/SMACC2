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

#include <nav2z_client/client_behaviors/cb_rotate_look_at.hpp>
#include <nav2z_client/common.hpp>
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <nav2z_client/nav2z_client.hpp>

#include <rclcpp/parameter_client.hpp>

namespace cl_nav2z
{
CbRotateLookAt::CbRotateLookAt() {}

CbRotateLookAt::CbRotateLookAt(const geometry_msgs::msg::PoseStamped & lookAtPose)
: lookAtPose_(lookAtPose)
{
}

void CbRotateLookAt::onEntry()
{
  cl_nav2z::Pose * pose;
  this->requiresComponent(pose);

  pose->waitTransformUpdate(rclcpp::Rate(20));
  auto position = pose->toPoseMsg().position;

  if (lookAtPose_)
  {
    auto targetPosition = lookAtPose_->pose.position;
    double yaw_degrees =
      atan2(targetPosition.y - position.y, targetPosition.x - position.x) * 180.0 / M_PI;
    this->absoluteGoalAngleDegree = yaw_degrees;
  }

  CbAbsoluteRotate::onEntry();
}

}  // namespace cl_nav2z
