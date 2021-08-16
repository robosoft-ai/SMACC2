#pragma once
#include <smacc/smacc_orthogonal.h>
#include <ros_timer_client/cl_ros_timer.h>

namespace sm_dance_bot_strikes_back
{
class OrTimer : public smacc::Orthogonal<OrTimer>
{
public:
    virtual void onInitialize() override
    {
        auto actionclient = this->createClient<cl_ros_timer::ClRosTimer>(rclcpp::Duration(0.5s));
        actionclient->initialize();
    }
};
}