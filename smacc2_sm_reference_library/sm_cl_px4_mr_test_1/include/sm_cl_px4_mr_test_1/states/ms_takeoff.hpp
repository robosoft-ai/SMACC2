#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

// MODE STATE: Vehicle is taking off
struct MsTakeoff
: smacc2::SmaccState<MsTakeoff, SmClPx4MrTest1, StTakeoff>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsTakeoff ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsTakeoff ---");
  }
};

}  // namespace sm_cl_px4_mr_test_1
