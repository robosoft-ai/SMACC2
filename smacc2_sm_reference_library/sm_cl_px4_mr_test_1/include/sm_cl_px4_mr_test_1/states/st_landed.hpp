#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

using namespace smacc2::default_transition_tags;

// STATE: Mission complete - terminal state
struct StLanded : smacc2::SmaccState<StLanded, MsLanded>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StLanded: MISSION COMPLETE");
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_1
