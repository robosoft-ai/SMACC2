#pragma once

#include <sm_dance_bot_strikes_back/clients/cl_lidar/cl_lidar.h>
#include <sm_dance_bot_strikes_back/clients/cl_lidar/components/cp_lidar_data.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot_strikes_back
{
class OrObstaclePerception : public smacc::Orthogonal<OrObstaclePerception>
{
public:
    virtual void onInitialize() override
    {
        auto lidarClient =
            this->createClient<ClLidarSensor>("/scan", rclcpp::Duration(10s));

        lidarClient->createComponent<CpLidarSensorData>();
    }
};
}