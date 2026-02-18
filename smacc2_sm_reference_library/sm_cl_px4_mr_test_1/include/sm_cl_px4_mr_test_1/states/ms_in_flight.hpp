#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

// MODE STATE: Vehicle is airborne and executing mission
struct MsInFlight
: smacc2::SmaccState<MsInFlight, SmClPx4MrTest1, StGoToWaypoint1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsInFlight ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsInFlight ---");
  }
};

}  // namespace sm_cl_px4_mr_test_1
