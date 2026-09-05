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

// STATE: Hold position while the heading sweeps sinusoidally around the entry
// heading. CbYawScan is continuous (never posts success) - the 30 s timer in
// OrTimer provides the exit event; on exit the behavior restores the base heading.
struct StYawScan : smacc2::SmaccState<StYawScan, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StReturnToBase, SUCCESS>,
    // entry-validity failure leg (no valid local position) -> abort home
    Transition<EvCbFailure<CbYawScan, OrPx4>, StReturnToBase, ABORT>
  > reactions;

  static void staticConfigure()
  {
    // +/-0.6 rad (~35 deg) sweep around the entry heading, 6 s period:
    // 30 s = 5 full sweep cycles
    configure_orthogonal<OrPx4, CbYawScan>(0.6f, 6.0f);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(30s);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StYawScan: scanning heading for 30 s...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StYawScan: scan window elapsed");
  }
};

}  // namespace sm_cl_px4_mr_test_3
