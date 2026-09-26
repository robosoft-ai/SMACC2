// Copyright 2026 RobosoftAI Inc.
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

#pragma once

#include <cl_generic_sensor/cl_generic_sensor.hpp>
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <cl_px4_mr/components/cp_tunnel_centering.hpp>
#include <config/mission_constants.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

// The 3D lidar: a PointCloud2 subscriber with a message watchdog
// (CpMessageTimeout -> EvTopicMessageTimeout<ClLidar, OrLidar>) and the
// forward safety cone (CpForwardObstacleGuard -> CbObstacleGuard events) and
// the tunnel centering the path follower adds to its carrots (CpTunnelCentering)
class OrLidar : public smacc2::Orthogonal<OrLidar>
{
public:
  void onInitialize() override
  {
    using ClLidar = cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::PointCloud2>;
    auto client = this->createClient<ClLidar>(std::string(kLidarTopic), rclcpp::Duration(kLidarTimeout));
    client->createComponent<cl_px4_mr::CpForwardObstacleGuard>(coneParams());
    client->createComponent<cl_px4_mr::CpTunnelCentering>(centeringParams());
  }
};

}  // namespace sm_cl_px4_mr_test_5
