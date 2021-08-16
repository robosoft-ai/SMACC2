#pragma once

#include <sm_dance_bot/clients/cl_lidar/cl_lidar.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot
{
class OrObstaclePerception : public smacc::Orthogonal<OrObstaclePerception>
{
public:
  virtual void onInitialize() override
  {
    auto lidarClient = this->createClient<ClLidarSensor>();

    lidarClient->topicName = "/scan";
    lidarClient->timeout_ = rclcpp::Duration(std::chrono::seconds(10));
  }
};
}  // namespace sm_dance_bot
