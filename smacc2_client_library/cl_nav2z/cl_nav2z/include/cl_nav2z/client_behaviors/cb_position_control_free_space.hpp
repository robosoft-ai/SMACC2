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

#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{
struct CbPositionControlFreeSpace : public smacc2::SmaccAsyncClientBehavior
{
private:
  double targetYaw_;
  bool goalReached_;
  double k_betta_;
  double max_angular_yaw_speed_;

  double prev_error_linear_ = 0.0;
  double prev_error_angular_ = 0.0;
  double integral_linear_ = 0.0;
  double integral_angular_ = 0.0;

  // Limit the maximum linear velocity and angular velocity to avoid sudden
  // movements
  double max_linear_velocity = 1.0;   // Adjust this value according to your needs
  double max_angular_velocity = 1.0;  // Adjust this value according to your needs

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:
  double yaw_goal_tolerance_rads_ = 0.1;

  double threshold_distance_ = 3.0;

  geometry_msgs::msg::Pose target_pose_;

  CbPositionControlFreeSpace();

  void updateParameters();

  void onEntry() override;

  void onExit() override;
};
}  // namespace cl_nav2z
