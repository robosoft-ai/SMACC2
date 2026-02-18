#include <sm_cl_px4_mr_test_1/sm_cl_px4_mr_test_1.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<sm_cl_px4_mr_test_1::SmClPx4MrTest1>();
}
