#include <ros_timer_client/cl_ros_timer.h>
#include <smacc/smacc.h>
#include <chrono>

using namespace std::chrono_literals;

namespace sm_atomic
{
using namespace std::chrono_literals;
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
  virtual void onInitialize() override
  {
    auto client = this->createClient<cl_ros_timer::ClRosTimer>(1s);
  }
};
}  // namespace sm_atomic