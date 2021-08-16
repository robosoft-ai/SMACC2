#pragma once

#include <ros_publisher_client/cl_ros_publisher.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot_strikes_back
{
class OrUpdatablePublisher : public smacc::Orthogonal<OrUpdatablePublisher>
{
public:
  virtual void onInitialize() override
  {
    auto publisherClient_ = this->createClient<cl_ros_publisher::ClRosPublisher>();
    publisherClient_->configure<std_msgs::msg::String>("/updatable_string_publisher_out");
  }
};
}  // namespace sm_dance_bot_strikes_back