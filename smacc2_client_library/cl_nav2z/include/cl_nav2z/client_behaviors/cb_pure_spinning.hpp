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

#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{
struct CbPureSpinning : public smacc2::SmaccAsyncClientBehavior
{
private:
  double targetYaw__rads;
  bool goalReached_;
  double k_betta_;
  double max_angular_yaw_speed_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:
  double yaw_goal_tolerance_rads_;

  CbPureSpinning(double targetYaw_rads, double max_angular_yaw_speed = 0.5);
  void updateParameters();

  void onEntry() override;

  void onExit() override;
};
}  // namespace cl_nav2z
