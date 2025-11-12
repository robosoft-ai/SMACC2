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

#include <angles/angles.h>
#include <cl_nav2z/client_behaviors/cb_pure_spinning.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
namespace cl_nav2z
{
using namespace smacc2;
CbPureSpinning::CbPureSpinning(double targetYaw_rads, double max_angular_yaw_speed)
: targetYaw__rads(targetYaw_rads),
  k_betta_(1.0),
  max_angular_yaw_speed_(max_angular_yaw_speed),
  yaw_goal_tolerance_rads_(0.1)
{
}

void CbPureSpinning::updateParameters() {}

void CbPureSpinning::onEntry()
{
  auto nh = this->getNode();
  cmd_vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1));

  cl_nav2z::CpPose * pose;
  this->requiresComponent(pose, ComponentRequirement::HARD);

  geometry_msgs::msg::Twist cmd_vel;
  goalReached_ = false;

  geometry_msgs::msg::Pose currentPose = pose->toPoseMsg();

  rclcpp::Rate loop_rate(10);
  double countAngle = 0;
  auto prevyaw = tf2::getYaw(currentPose.orientation);
  while (rclcpp::ok() && !goalReached_)
  {
    tf2::Quaternion q;
    currentPose = pose->toPoseMsg();
    tf2::fromMsg(currentPose.orientation, q);
    auto currentYaw = tf2::getYaw(currentPose.orientation);
    auto deltaAngle = angles::shortest_angular_distance(prevyaw, currentYaw);
    countAngle += deltaAngle;

    prevyaw = currentYaw;
    double angular_error = targetYaw__rads - countAngle;

    auto omega = angular_error * k_betta_;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.z =
      std::min(std::max(omega, -fabs(max_angular_yaw_speed_)), fabs(max_angular_yaw_speed_));

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] delta angle: " << deltaAngle);
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] cummulated angle: " << countAngle);
    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] k_betta_: " << k_betta_);

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] angular error: " << angular_error << "("
                       << yaw_goal_tolerance_rads_ << ")");
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] command angular speed: " << cmd_vel.angular.z);

    if (fabs(angular_error) < yaw_goal_tolerance_rads_)
    {
      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] GOAL REACHED. Sending stop command.");
      goalReached_ = true;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      break;
    }

    this->cmd_vel_pub_->publish(cmd_vel);

    loop_rate.sleep();
  }

  this->postSuccessEvent();
}

void CbPureSpinning::onExit() {}
}  // namespace cl_nav2z
