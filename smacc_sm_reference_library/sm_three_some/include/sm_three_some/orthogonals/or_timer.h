#pragma once

#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
using namespace std::chrono_literals;
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
  virtual void onInitialize() override
  {
    auto actionclient =
        this->createClient<cl_ros_timer::ClRosTimer>(500ms);
  }
};
}  // namespace sm_three_some