#include <smacc2/smacc.hpp>
#include <ros_timer_client/cl_ros_timer.hpp>
#include <chrono>

namespace sm_atomic_mode_states
{

using namespace std::chrono_literals;

class OrTimer : public smacc2::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto client = this->createClient<cl_ros_timer::ClRosTimer>(rclcpp::Duration(1s));
    }
};
} // namespace sm_atomic_mode_states
