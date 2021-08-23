#pragma once

#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc_orthogonal.h>
#include <chrono>

namespace sm_respira_1
{
using namespace std::chrono_literals;
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
  virtual void onInitialize() override
  {
    auto actionclient = this->createClient<cl_ros_timer::ClRosTimer>(rclcpp::Duration(0.1s));
  }
};
}  // namespace sm_respira_1
