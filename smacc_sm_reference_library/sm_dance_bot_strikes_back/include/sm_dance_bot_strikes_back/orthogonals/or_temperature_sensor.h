#pragma once

#include <multirole_sensor_client/cl_multirole_sensor.h>
#include <sm_dance_bot_strikes_back/clients/cl_temperature_sensor/cl_temperature_sensor.h>
#include <smacc/smacc_orthogonal.h>
#include <sensor_msgs/msg/temperature.hpp>

using namespace std::chrono_literals;

namespace sm_dance_bot_strikes_back
{
class OrTemperatureSensor : public smacc::Orthogonal<OrTemperatureSensor>
{
public:
  virtual void onInitialize() override
  {
    auto clTemperatureSensor = this->createClient<ClTemperatureSensor>();

    clTemperatureSensor->topicName = "/temperature";
    // ClTemperatureSensor->queueSize = "/front/scan";
    clTemperatureSensor->timeout_ = rclcpp::Duration(10s);

    // clTemperatureSensor->initialize();
  }
};
}  // namespace sm_dance_bot_strikes_back
