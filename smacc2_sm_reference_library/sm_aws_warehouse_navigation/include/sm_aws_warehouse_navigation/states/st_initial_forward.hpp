#pragma once
#include <smacc2/smacc.hpp>

namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StInitialNavigateForward : smacc2::SmaccState<StInitialNavigateForward, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

  Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateMainAisle>
  >
  reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(2);
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "Hello world!"); }

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
