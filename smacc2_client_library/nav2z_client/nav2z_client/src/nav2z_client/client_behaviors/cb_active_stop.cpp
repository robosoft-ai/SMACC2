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

#include <nav2z_client/client_behaviors/cb_active_stop.hpp>

namespace cl_nav2z
{
CbActiveStop::CbActiveStop() {}

void CbActiveStop::onEntry()
{
  auto nh = this->getNode();
  cmd_vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1));

  rclcpp::Rate loop_rate(5);
  geometry_msgs::msg::Twist cmd_vel_msg;
  while (!this->isShutdownRequested())
  {
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;

    cmd_vel_pub_->publish(cmd_vel_msg);
    loop_rate.sleep();
  }
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Finished behavior execution");

  this->postSuccessEvent();
}

void CbActiveStop::onExit() {}

}  // namespace cl_nav2z
