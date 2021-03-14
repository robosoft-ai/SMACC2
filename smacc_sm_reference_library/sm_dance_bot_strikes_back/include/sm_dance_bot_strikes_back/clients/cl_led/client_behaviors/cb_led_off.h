#pragma once

#include <smacc/smacc.h>
#include <sm_dance_bot_strikes_back/clients/cl_led/cl_led.h>

namespace sm_dance_bot_strikes_back
{
namespace cl_led
{
class CbLEDOff : public smacc::SmaccClientBehavior
{
public:
  cl_led::ClLED *ledActionClient_;

  virtual void onEntry() override
  {
    this->requiresClient(ledActionClient_);

    cl_led::ClLED::Goal goal;
    goal.command = cl_led::ClLED::Goal::CMD_OFF;
    ledActionClient_->sendGoal(goal);
  }

  virtual void onExit() override
  {
    //RCLCPP_INFO(nh_->get_logger(), "Entering ToolClientBehavior");
  }
};
} // namespace cl_led
} // namespace sm_dance_bot_strikes_back
