#include <sm_dance_bot_strikes_back/sm_dance_bot_strikes_back.h>
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  smacc::run<SmDanceBotStrikesBack>();
}