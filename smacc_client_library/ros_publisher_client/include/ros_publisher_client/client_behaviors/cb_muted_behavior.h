
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <ros_publisher_client/cl_ros_publisher.h>

namespace cl_ros_publisher
{
template <typename RosMsgType>
class CbMutedBehavior : public smacc::SmaccClientBehavior
{
public:
    virtual void onEntry() override
    {
    }
    virtual void onExit() override
    {
    }
};
} // namespace cl_ros_publisher
