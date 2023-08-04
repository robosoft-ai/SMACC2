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

#include <angles/angles.h>
#include <geometry_msgs/msg/twist.hpp>
#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace sm_dancebot_ue
{
struct CbActiveStop : public smacc2::SmaccAsyncClientBehavior
{
private:


  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:

  CbActiveStop();

  void onEntry() override;

  void onExit() override;
};
}  // namespace sm_dancebot_ue
