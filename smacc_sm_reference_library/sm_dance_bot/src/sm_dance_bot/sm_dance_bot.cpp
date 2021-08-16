#include <sm_dance_bot/sm_dance_bot.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc::run<SmDanceBot>();
}
