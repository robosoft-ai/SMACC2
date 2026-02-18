#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

// MODE STATE: Vehicle is being armed
struct MsArmedOnGround
: smacc2::SmaccState<MsArmedOnGround, SmClPx4MrTest1, StArmPx4>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsArmedOnGround ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsArmedOnGround ---");
  }
};

}  // namespace sm_cl_px4_mr_test_1
