#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

// MODE STATE: Vehicle has landed, mission complete
struct MsLanded
: smacc2::SmaccState<MsLanded, SmClPx4MrTest1, StLanded>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsLanded ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsLanded ---");
  }
};

}  // namespace sm_cl_px4_mr_test_1
