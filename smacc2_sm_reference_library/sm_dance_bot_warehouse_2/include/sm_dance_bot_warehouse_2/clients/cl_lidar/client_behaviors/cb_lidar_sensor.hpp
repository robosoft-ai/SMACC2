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
#include <sm_dance_bot_warehouse_2/clients/cl_lidar/cl_lidar.hpp>

namespace sm_dance_bot_warehouse_2
{
namespace cl_lidar
{
struct CbLidarSensor : cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<ClLidarSensor>
{
public:
  CbLidarSensor() {}

  void onEntry() override
  {
    RCLCPP_INFO(getLogger(), "[CbLidarSensor] onEntry");
    cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<ClLidarSensor>::onEntry();
  }

  virtual void onMessageCallback(const sensor_msgs::msg::LaserScan & /*msg*/) override {}
};
}  // namespace cl_lidar
}  // namespace sm_dance_bot_warehouse_2
