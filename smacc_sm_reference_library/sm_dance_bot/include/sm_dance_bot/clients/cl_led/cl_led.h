#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <sm_dance_bot_msgs/action/led_control.hpp>

namespace sm_dance_bot
{
namespace cl_led
{
// class ClLED: public smacc::client_bases::ISmaccActionClient
// {
//     public:
//         ClLED(std::string actionServerName);
//         virtual std::string getName() const override;
//         virtual ~ClLED();
//         virtual void cancelGoal() override; 
// };


class ClLED : public smacc::client_bases::SmaccActionClientBase<sm_dance_bot_msgs::action::LEDControl>
{

public:
    // SMACC_ACTION_CLIENT_DEFINITION(sm_dance_bot_msgs::action::LEDControlAction);

    ClLED(std::string actionServerName);
    virtual std::string getName() const override;
    virtual ~ClLED();
};

std::ostream& operator<< (std::ostream &out, const sm_dance_bot_msgs::action::LEDControl::Goal &msg);

} // namespace cl_led
}