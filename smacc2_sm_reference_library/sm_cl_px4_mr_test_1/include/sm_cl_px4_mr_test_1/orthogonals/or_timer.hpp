#pragma once

#include <smacc2/smacc.hpp>
#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <chrono>

namespace sm_cl_px4_mr_test_1
{

using namespace std::chrono_literals;

class OrTimer : public smacc2::Orthogonal<OrTimer>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_ros2_timer::ClRos2Timer>(rclcpp::Duration(1s));
  }
};

}  // namespace sm_cl_px4_mr_test_1
