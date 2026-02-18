#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Return to launch position
struct StReturnToBase : smacc2::SmaccState<StReturnToBase, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, MsLanding, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Return to origin (0, 0, -5) NED = home position at 5m altitude
    configure_orthogonal<OrPx4, CbGoToLocation>(0.0f, 0.0f, -5.0f);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StReturnToBase: returning to base...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StReturnToBase: back at base");
  }
};

}  // namespace sm_cl_px4_mr_test_1
