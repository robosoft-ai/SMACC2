#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Arm the vehicle
struct StArmPx4 : smacc2::SmaccState<StArmPx4, MsArmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbArmPX4, OrPx4>, MsTakeoff, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbArmPX4>();
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StArmPx4: arming vehicle...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StArmPx4: vehicle armed");
  }
};

}  // namespace sm_cl_px4_mr_test_1
