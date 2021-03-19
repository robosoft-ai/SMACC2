#pragma once

#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_ferrari
{
using std::chrono_literals;
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
  virtual void onInitialize() override
  {
    auto actionclient =
        this->createClient<cl_ros_timer::ClRosTimer>(std::chrono::duration_cast<std::chrono::seconds>(500ms));
    actionclient->initialize();
  }
};
}  // namespace sm_ferrari