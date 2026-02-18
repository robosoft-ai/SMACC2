#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;

// STATE: Wait for PX4 topics to come up before proceeding
struct StWaitForReady : smacc2::SmaccState<StWaitForReady, MsDisarmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, MsArmedOnGround, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Wait 5 seconds for PX4 topics to stabilize
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StWaitForReady: waiting for PX4 topics...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StWaitForReady: ready, proceeding to arm");
  }
};

}  // namespace sm_cl_px4_mr_test_1
