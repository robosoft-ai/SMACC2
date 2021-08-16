#pragma once

#include <ros_publisher_client/cl_ros_publisher.h>
#include <smacc/smacc_orthogonal.h>
#include <std_msgs/msg/string.hpp>

namespace sm_three_some
{
using namespace cl_ros_publisher;

class OrUpdatablePublisher : public smacc::Orthogonal<OrUpdatablePublisher>
{
public:
  void onInitialize() override { auto ros_publisher_client = this->createClient<ClRosPublisher>(); }
};
}  // namespace sm_three_some
