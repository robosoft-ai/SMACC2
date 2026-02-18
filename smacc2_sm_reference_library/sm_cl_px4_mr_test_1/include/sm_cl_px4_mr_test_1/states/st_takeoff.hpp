#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Take off to target altitude
struct StTakeoff : smacc2::SmaccState<StTakeoff, MsTakeoff>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbTakeOff, OrPx4>, MsInFlight, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Take off to 5 meters altitude
    configure_orthogonal<OrPx4, CbTakeOff>(5.0f);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StTakeoff: taking off...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StTakeoff: takeoff complete");
  }
};

}  // namespace sm_cl_px4_mr_test_1
