#include <sm_dance_bot/sm_dance_bot.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("sm_dance_bot");

    rclcpp::sleep_for(std::chrono::seconds(5));
    smacc::run<SmDanceBot>();
}
