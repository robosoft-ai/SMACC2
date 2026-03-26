// Copyright 2025 Robosoft Inc.
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

namespace sm_cl_px4_mr_test_2
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

}  // namespace sm_cl_px4_mr_test_2
