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

#include <cmath>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_2
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Rotate 90 degrees to the right (relative yaw)
struct StYawRotate : smacc2::SmaccState<StYawRotate, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbYawRotate, OrPx4>, StHoldPosition2, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Rotate 90 degrees (PI/2 radians) relative to current heading
    configure_orthogonal<OrPx4, CbYawRotate>(static_cast<float>(M_PI / 2.0), true);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StYawRotate: rotating 90 degrees...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StYawRotate: rotation complete");
  }
};

}  // namespace sm_cl_px4_mr_test_2
