
#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <std_msgs/msg/float32.hpp>

namespace sm_ferrari
{
namespace cl_subscriber
{
class ClSubscriber : public smacc::client_bases::SmaccSubscriberClient<std_msgs::msg::Float32>
{
public:
  ClSubscriber(std::string topicname) : smacc::client_bases::SmaccSubscriberClient<std_msgs::msg::Float32>(topicname)
  {
  }
};
}  // namespace cl_subscriber
}  // namespace sm_ferrari