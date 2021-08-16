#pragma once
#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
  void onInitialize() override
  {
    auto actionclient = this->createClient<cl_ros_timer::ClRosTimer>(
      rclcpp::Duration(std::chrono::milliseconds(500)));

    actionclient->initialize();
  }
};
}  // namespace sm_dance_bot
