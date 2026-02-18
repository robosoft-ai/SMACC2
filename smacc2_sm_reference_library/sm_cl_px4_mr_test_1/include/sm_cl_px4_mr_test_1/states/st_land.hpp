#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Land the vehicle
struct StLand : smacc2::SmaccState<StLand, MsLanding>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLand, OrPx4>, MsLanded, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLand>();
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StLand: landing...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StLand: landed");
  }
};

}  // namespace sm_cl_px4_mr_test_1
