#pragma once

#include <sm_dance_bot_strikes_back/clients/cl_string_publisher/cl_string_publisher.h>
#include <smacc/smacc_orthogonal.h>
namespace sm_dance_bot_strikes_back
{
class OrStringPublisher : public smacc::Orthogonal<OrStringPublisher>
{
public:
    virtual void onInitialize() override
    {
        this->createClient<ClStringPublisher>("/string_publisher_out");
    }
};
} // namespace sm_dance_bot_strikes_back