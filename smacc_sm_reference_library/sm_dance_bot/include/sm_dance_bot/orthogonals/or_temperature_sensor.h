#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sensor_msgs/msg/temperature.hpp>
#include <multirole_sensor_client/cl_multirole_sensor.h>
#include <sm_dance_bot/clients/cl_temperature_sensor/cl_temperature_sensor.h>

namespace sm_dance_bot
{
class OrTemperatureSensor : public smacc::Orthogonal<OrTemperatureSensor>
{
public:
    virtual void onInitialize() override
    {
        auto clTemperatureSensor = this->createClient<ClTemperatureSensor>();

        clTemperatureSensor->topicName = "/temperature";
        //ClTemperatureSensor->queueSize = "/front/scan";
        clTemperatureSensor->timeout_ = rclcpp::Duration(10);

        //clTemperatureSensor->initialize();
    }
};
} // namespace sm_dance_bot