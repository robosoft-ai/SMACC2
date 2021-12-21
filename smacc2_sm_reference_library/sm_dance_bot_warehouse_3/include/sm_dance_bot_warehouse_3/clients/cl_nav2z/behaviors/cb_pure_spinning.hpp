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

#pragma once

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sm_dance_bot_warehouse_3/clients/cl_lidar/cl_lidar.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <angles/angles.h>

namespace sm_dance_bot_warehouse_3
{
namespace cl_nav2zclient
{
struct CbPureSpinning : public smacc2::SmaccAsyncClientBehavior
{
  private:
    double targetYaw_;
    bool goalReached_;
    double k_betta_;
    double max_angular_yaw_speed_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  public:
    double yaw_goal_tolerance_rads_;

  CbPureSpinning(double targetYaw, double max_angular_yaw_speed= 0.5)
  : targetYaw_(targetYaw),
    k_betta_(1.0),
    max_angular_yaw_speed_(max_angular_yaw_speed),
    yaw_goal_tolerance_rads_(0.03)
  {

  }

  void updateParameters()
  {
  }

  void onEntry() override
  {
      auto nh = this->getNode();
      cmd_vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

      cl_nav2z::Pose* pose;
      this->requiresComponent(pose);

      geometry_msgs::msg::Twist cmd_vel;
      goalReached_ = false;

      geometry_msgs::msg::PoseStamped currentPose = pose->toPoseStampedMsg();

      rclcpp::Rate loop_rate(10);
      double countAngle = 0;
      auto prevyaw = tf2::getYaw(currentPose.pose.orientation);
      while(rclcpp::ok() && !goalReached_)
      {
        tf2::Quaternion q;
        currentPose = pose->toPoseStampedMsg();
        tf2::fromMsg(currentPose.pose.orientation, q);
        auto currentYaw = tf2::getYaw(currentPose.pose.orientation);
        auto deltaAngle = angles::shortest_angular_distance(prevyaw, currentYaw);
        countAngle += deltaAngle;

        prevyaw = currentYaw;
        double angular_error = targetYaw_ - countAngle ;

        auto omega = angular_error * k_betta_;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        cmd_vel.angular.z =
          std::min(std::max(omega, -fabs(max_angular_yaw_speed_)), fabs(max_angular_yaw_speed_));

        RCLCPP_INFO_STREAM(getLogger(), "["<<getName() << "] delta angle: " << deltaAngle);
        RCLCPP_INFO_STREAM(getLogger(), "["<<getName() << "] cummulated angle: " << countAngle);
        RCLCPP_INFO_STREAM(getLogger(), "["<<getName() << "] k_betta_: " << k_betta_);

        RCLCPP_INFO_STREAM(
          getLogger(), "["<<getName() << "] angular error: " << angular_error << "("
                                                                          << yaw_goal_tolerance_rads_ << ")");
        RCLCPP_INFO_STREAM(
          getLogger(),
          "["<<getName() << "] command angular speed: " << cmd_vel.angular.z);

        if (fabs(angular_error) < yaw_goal_tolerance_rads_)
        {
          RCLCPP_INFO_STREAM(
            getLogger(), "["<<getName() << "] GOAL REACHED. Sending stop command.");
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

  void onExit() override
  {
  }
  };
}
}
