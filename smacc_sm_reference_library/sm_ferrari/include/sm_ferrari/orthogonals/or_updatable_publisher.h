#pragma once

#include <ros_publisher_client/cl_ros_publisher.h>
#include <smacc/smacc_orthogonal.h>
#include <std_msgs/String.h>

namespace sm_ferrari
{
using namespace cl_ros_publisher;

class OrUpdatablePublisher : public smacc::Orthogonal<OrUpdatablePublisher>
{
public:
    virtual void onInitialize() override
    {
        auto ros_publisher_client = this->createClient<ClRosPublisher>();
        ros_publisher_client->initialize();
    }
};
} // namespace sm_ferrari