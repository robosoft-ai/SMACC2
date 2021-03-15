#pragma once

#include <multirole_sensor_client/cl_multirole_sensor.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot
{
namespace cl_lidar
{
class ClLidarSensor : public cl_multirole_sensor::ClMultiroleSensor<sensor_msgs::msg::LaserScan>
{
public:
  ClLidarSensor()
  {
  }
};
}  // namespace cl_lidar
}  // namespace sm_dance_bot