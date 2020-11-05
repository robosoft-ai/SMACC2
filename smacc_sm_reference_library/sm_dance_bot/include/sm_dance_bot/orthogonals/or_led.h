#pragma once
#include <smacc/smacc_orthogonal.h>
#include <sm_dance_bot/clients/cl_led/cl_led.h>

namespace sm_dance_bot
{
class OrLED : public smacc::Orthogonal<OrLED>
{
public:
    virtual void onInitialize() override
    {
        auto actionclient = this->createClient<cl_led::ClLED>("led_action_server");
        actionclient->initialize();
    }
};
} // namespace sm_dance_bot