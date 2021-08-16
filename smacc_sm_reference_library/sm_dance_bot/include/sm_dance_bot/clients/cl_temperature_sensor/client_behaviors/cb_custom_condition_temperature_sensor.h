
#pragma once
#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>
#include <sm_dance_bot/clients/cl_temperature_sensor/cl_temperature_sensor.h>
#include <sensor_msgs/msg/temperature.hpp>

namespace sm_dance_bot
{
namespace cl_temperature_sensor
{
struct EvCustomTemperatureAlert : sc::event<EvCustomTemperatureAlert>
{
};

//--------------------------------------------------------------------------------------
class CbConditionTemperatureSensor
: public cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<ClTemperatureSensor>
{
public:
  CbConditionTemperatureSensor() {}
  virtual void onMessageCallback(const sensor_msgs::msg::Temperature & msg) override
  {
    if (msg.temperature > 40)
    {
      auto ev = new EvCustomTemperatureAlert();
      this->postEvent(ev);
    }
  }
};
}  // namespace cl_temperature_sensor
}  // namespace sm_dance_bot
