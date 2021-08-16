
#include <smacc/smacc.h>

namespace sm_dance_bot_strikes_back
{
namespace cl_lidar
{
using namespace std::chrono_literals;

  void CpLidarSensorData::onInitialize()
  {
    auto client_ = dynamic_cast<smacc::client_bases::SmaccSubscriberClient<sensor_msgs::msg::LaserScan> *>(owner_);
    client_->onMessageReceived(&CpLidarSensorData::MessageCallbackStoreDistanceToWall, this);
  }

  void CpLidarSensorData::MessageCallbackStoreDistanceToWall(const sensor_msgs::msg::LaserScan &scanmsg)
  {
    this->lastMessage_ = scanmsg;
    //auto fwdist = scanmsg.ranges[scanmsg.ranges.size() / 2] /*meter*/;
    auto fwdist = scanmsg.ranges[0] /*meter*/;

    /*if the distance is infinity or nan, use max range distance*/
    if (fwdist == std::numeric_limits<float>::infinity() || fwdist != fwdist)
    {
      this->forwardObstacleDistance = scanmsg.range_max - SECURITY_DISTANCE /*meters*/;
      RCLCPP_INFO_THROTTLE(getNode()->get_logger(), *(getNode()->get_clock()), 1000,
                           "[CpLidarSensorData] Distance to forward obstacle is not a number, setting default value "
                           "to: %lf",
                           scanmsg.range_max);
    }
    else
    {
      this->forwardObstacleDistance = std::max(fwdist - SECURITY_DISTANCE, 0.0F);
    }
  }

}  // namespace cl_lidar
}  // namespace sm_dance_bot_strikes_back