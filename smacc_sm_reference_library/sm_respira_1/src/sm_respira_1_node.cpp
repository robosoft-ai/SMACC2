#include <sm_respira_1/sm_respira_1.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc::run<sm_respira_1::SmRespira1>();
}
