#include <sm_atomic_performance_test/sm_atomic_performance_test.h>

//--------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc::run<sm_atomic_performance_test::SmAtomicPerformanceTest>();
}
