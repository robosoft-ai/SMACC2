#include <smacc/smacc.h>
#include <ros_timer_client/cl_ros_timer.h>
#include <chrono>

using namespace std::chrono_literals;

namespace sm_atomic
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<cl_ros_timer::ClRosTimer>(1s);
        client->initialize();
    }
};
} // namespace sm_atomic