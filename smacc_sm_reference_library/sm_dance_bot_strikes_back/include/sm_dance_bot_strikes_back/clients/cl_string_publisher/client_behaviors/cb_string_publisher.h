#pragma once

#include <sm_dance_bot_strikes_back/clients/cl_string_publisher/cl_string_publisher.h>
#include <smacc/smacc_client_behavior.h>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot_strikes_back
{
namespace cl_string_publisher
{
class CbStringPublisher : public smacc::SmaccClientBehavior
{
public:
    ClStringPublisher *publisherClient_;
    std::string msg_;

    CbStringPublisher(std::string msg)
    {
        msg_ = msg;
    }

    virtual void runtimeConfigure() override
    {
        RCLCPP_INFO_STREAM(getNode()->get_logger(),"Creating CbStringPublisher behavior with stored message: " << msg_);
    }

    virtual void onEntry()
    {
        this->requiresClient(publisherClient_);
    }

    virtual void onExit() override
    {
        std_msgs::msg::String rosmsg;
        rosmsg.data = msg_;
        publisherClient_->publish(rosmsg);
    }
};
} // namespace cl_string_publisher
} // namespace sm_dance_bot
