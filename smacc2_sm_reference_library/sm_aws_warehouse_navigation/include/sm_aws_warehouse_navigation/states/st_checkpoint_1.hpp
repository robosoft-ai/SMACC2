#include <smacc2/smacc.hpp>

namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StCheckPoint1 : smacc2::SmaccState<StCheckPoint1, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE

  // STATE FUNCTIONS
  static void staticConfigure() 
  {
        configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-2,-3,0);
  }

  void runtimeConfigure() {}

  void onEntry() {}

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
