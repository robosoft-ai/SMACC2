#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot
{
namespace cl_string_publisher
{
class ClStringPublisher : public smacc::client_bases::SmaccPublisherClient
{
public:
    ClStringPublisher(std::string topicName)
        : smacc::client_bases::SmaccPublisherClient()
    {
        this->configure<std_msgs::msg::String>(topicName);
    }
};
} // namespace cl_string_publisher
} // namespace sm_dance_bot