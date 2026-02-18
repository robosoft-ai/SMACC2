#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

// MODE STATE: Vehicle is disarmed on the ground, waiting for readiness
struct MsDisarmedOnGround
: smacc2::SmaccState<MsDisarmedOnGround, SmClPx4MrTest1, StWaitForReady>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsDisarmedOnGround ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsDisarmedOnGround ---");
  }
};

}  // namespace sm_cl_px4_mr_test_1
