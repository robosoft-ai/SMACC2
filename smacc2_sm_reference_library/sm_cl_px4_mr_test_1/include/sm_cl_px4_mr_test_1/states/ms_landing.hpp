#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

// MODE STATE: Vehicle is landing
struct MsLanding
: smacc2::SmaccState<MsLanding, SmClPx4MrTest1, StLand>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsLanding ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsLanding ---");
  }
};

}  // namespace sm_cl_px4_mr_test_1
