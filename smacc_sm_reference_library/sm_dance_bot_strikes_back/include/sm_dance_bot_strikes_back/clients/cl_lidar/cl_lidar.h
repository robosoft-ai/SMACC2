#pragma once

#include <multirole_sensor_client/cl_multirole_sensor.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot_strikes_back
{
namespace cl_lidar
{
class ClLidarSensor : public cl_multirole_sensor::ClMultiroleSensor<sensor_msgs::msg::LaserScan>
{
public:
  ClLidarSensor(std::string topicname, rclcpp::Duration timeout)
  {
    this->topicName = topicname;
    this->timeout_ = timeout;
  }
};
}  // namespace cl_lidar
}  // namespace sm_dance_bot_strikes_back