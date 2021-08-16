#pragma once

#include <multirole_sensor_client/cl_multirole_sensor.h>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot_strikes_back
{
namespace cl_temperature_sensor
{
class ClTemperatureSensor
: public cl_multirole_sensor::ClMultiroleSensor<sensor_msgs::msg::Temperature>
{
public:
  ClTemperatureSensor() {}
};
}  // namespace cl_temperature_sensor
}  // namespace sm_dance_bot_strikes_back