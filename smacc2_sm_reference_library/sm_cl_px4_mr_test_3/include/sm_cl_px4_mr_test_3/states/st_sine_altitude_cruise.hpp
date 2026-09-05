// Copyright 2026 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_3
{

using namespace cl_px4_mr;
using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;

// STATE: Cruise along the current heading with sinusoidal altitude.
// CbSineAltitudeCruise is continuous (never posts success) - the 30 s timer in
// OrTimer provides the exit event.
struct StSineAltitudeCruise : smacc2::SmaccState<StSineAltitudeCruise, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StYawScan, SUCCESS>,
    // entry-validity failure leg (no valid local position) -> abort home
    Transition<EvCbFailure<CbSineAltitudeCruise, OrPx4>, StReturnToBase, ABORT>
  > reactions;

  static void staticConfigure()
  {
    // 2 m/s along the entry heading, +/-1.5 m sine around the entry altitude,
    // 20 m wavelength: 30 s = 60 m of track = 3 full sine cycles
    configure_orthogonal<OrPx4, CbSineAltitudeCruise>(2.0f, 1.5f, 20.0f);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(30s);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StSineAltitudeCruise: cruising with sine altitude for 30 s...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StSineAltitudeCruise: cruise window elapsed");
  }
};

}  // namespace sm_cl_px4_mr_test_3
