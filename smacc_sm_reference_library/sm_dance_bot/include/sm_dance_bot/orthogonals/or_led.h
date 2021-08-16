#pragma once
#include <sm_dance_bot/clients/cl_led/cl_led.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_dance_bot
{
class OrLED : public smacc::Orthogonal<OrLED>
{
public:
  void onInitialize() override
  {
    auto actionclient = this->createClient<cl_led::ClLED>("led_action_server");
  }
};
}  // namespace sm_dance_bot
