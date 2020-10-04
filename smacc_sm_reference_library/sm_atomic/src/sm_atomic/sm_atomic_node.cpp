#include <sm_atomic/sm_atomic.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    smacc::run<sm_atomic::SmAtomic>();
}