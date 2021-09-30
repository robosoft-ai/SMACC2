#pragma once
#include <smacc2/smacc.hpp>

namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StRotateMainAisle : smacc2::SmaccState<StRotateMainAisle, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE

  // STATE FUNCTIONS
  static void staticConfigure() 
  {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>(-90);
  }

  void runtimeConfigure() {}

  void onEntry() { }

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
