#include <sm_ferrari/sm_ferrari.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  smacc::run<sm_ferrari::SmFerrari>();
}
