#include <sm_three_some/sm_three_some.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc::run<sm_three_some::SmThreeSome>();
}
