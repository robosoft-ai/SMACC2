#include <sm_atomic_services/sm_atomic_services.hpp>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
     rclcpp::init(argc, argv);
     smacc2::run<sm_atomic_services::SmAtomicServices>();
}
