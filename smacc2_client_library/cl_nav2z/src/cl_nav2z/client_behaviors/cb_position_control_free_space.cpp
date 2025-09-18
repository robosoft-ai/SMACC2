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
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <cl_nav2z/client_behaviors/cb_position_control_free_space.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
using namespace smacc2;
CbPositionControlFreeSpace::CbPositionControlFreeSpace()
: targetYaw_(0), k_betta_(1.0), max_angular_yaw_speed_(1.0)
{
}

void CbPositionControlFreeSpace::updateParameters() {}

void CbPositionControlFreeSpace::onEntry()
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

  // PID controller gains (proportional, integral, and derivative)
  double kp_linear = 0.5;
  double ki_linear = 0.0;
  double kd_linear = 0.1;

  double kp_angular = 0.5;
  double ki_angular = 0.0;
  double kd_angular = 0.1;

  while (rclcpp::ok() && !goalReached_)
  {
    RCLCPP_INFO_STREAM_THROTTLE(
      getLogger(), *getNode()->get_clock(), 200,
      "CbPositionControlFreeSpace, current pose: " << currentPose.position.x << ", "
                                                   << currentPose.position.y << ", "
                                                   << tf2::getYaw(currentPose.orientation));

    RCLCPP_INFO_STREAM_THROTTLE(
      getLogger(), *getNode()->get_clock(), 200,
      "CbPositionControlFreeSpace, target pose: " << target_pose_.position.x << ", "
                                                  << target_pose_.position.y << ", "
                                                  << tf2::getYaw(target_pose_.orientation));

    tf2::Quaternion q;
    currentPose = pose->toPoseMsg();

    // Here we implement the control logic to calculate the velocity command
    // based on the received position of the vehicle and the target pose.

    // Calculate the errors in x and y
    double error_x = target_pose_.position.x - currentPose.position.x;
    double error_y = target_pose_.position.y - currentPose.position.y;

    // Calculate the distance to the target pose
    double distance_to_target = std::sqrt(error_x * error_x + error_y * error_y);

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] distance to target: " << distance_to_target
                       << " ( th: " << threshold_distance_ << ")");

    // Check if the robot has reached the target pose
    if (distance_to_target < threshold_distance_)
    {
      RCLCPP_INFO(getLogger(), "Goal reached!");
      // Stop the robot by setting the velocity commands to zero
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      break;
    }
    else
    {
      // Calculate the desired orientation angle
      double desired_yaw = std::atan2(error_y, error_x);

      // Calculate the difference between the desired orientation and the
      // current orientation
      double yaw_error = desired_yaw - (tf2::getYaw(currentPose.orientation) + M_PI);

      // Ensure the yaw error is within the range [-pi, pi]
      while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
      while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

      // Calculate the control signals (velocity commands) using PID controllers
      double cmd_linear_x = kp_linear * distance_to_target + ki_linear * integral_linear_ +
                            kd_linear * (distance_to_target - prev_error_linear_);
      double cmd_angular_z = kp_angular * yaw_error + ki_angular * integral_angular_ +
                             kd_angular * (yaw_error - prev_error_angular_);

      if (cmd_linear_x > max_linear_velocity)
        cmd_linear_x = max_linear_velocity;
      else if (cmd_linear_x < -max_linear_velocity)
        cmd_linear_x = -max_linear_velocity;

      if (cmd_angular_z > max_angular_velocity)
        cmd_angular_z = max_angular_velocity;
      else if (cmd_angular_z < -max_angular_velocity)
        cmd_angular_z = -max_angular_velocity;

      // Construct and publish the velocity command message
      geometry_msgs::msg::Twist cmd_vel_msg;

      cmd_vel_msg.linear.x = cmd_linear_x;
      cmd_vel_msg.angular.z = cmd_angular_z;

      cmd_vel_pub_->publish(cmd_vel_msg);

      // Update errors and integrals for the next control cycle
      prev_error_linear_ = distance_to_target;
      prev_error_angular_ = yaw_error;
      integral_linear_ += distance_to_target;
      integral_angular_ += yaw_error;

      // tf2::fromMsg(currentPose.orientation, q);
      // auto currentYaw = tf2::getYaw(currentPose.orientation);
      // auto deltaAngle = angles::shortest_angular_distance(prevyaw,
      // currentYaw); countAngle += deltaAngle;

      // prevyaw = currentYaw;
      // double angular_error = targetYaw_ - countAngle;

      // auto omega = angular_error * k_betta_;
      // cmd_vel.linear.x = 0;
      // cmd_vel.linear.y = 0;
      // cmd_vel.linear.z = 0;
      // cmd_vel.angular.z =
      //   std::min(std::max(omega, -fabs(max_angular_yaw_speed_)),
      //   fabs(max_angular_yaw_speed_));

      // RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] delta angle: "
      // << deltaAngle); RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "]
      // cummulated angle: " << countAngle); RCLCPP_INFO_STREAM(getLogger(), "["
      // << getName() << "] k_betta_: " << k_betta_);

      // RCLCPP_INFO_STREAM(
      //   getLogger(), "[" << getName() << "] angular error: " << angular_error
      //   << "("
      //                    << yaw_goal_tolerance_rads_ << ")");
      // RCLCPP_INFO_STREAM(
      //   getLogger(), "[" << getName() << "] command angular speed: " <<
      //   cmd_vel.angular.z);

      // if (fabs(angular_error) < yaw_goal_tolerance_rads_)
      // {
      //   RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] GOAL REACHED.
      //   Sending stop command."); goalReached_ = true; cmd_vel.linear.x = 0;
      //   cmd_vel.angular.z = 0;
      //   break;
      // }

      // this->cmd_vel_pub_->publish(cmd_vel);

      loop_rate.sleep();
    }
  }

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Finished behavior execution");

  this->postSuccessEvent();
}

void CbPositionControlFreeSpace::onExit() {}

}  // namespace cl_nav2z
