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

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Fly an expanding spiral pattern (search and rescue)
struct StSpiralPattern : smacc2::SmaccState<StSpiralPattern, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSpiralPattern, OrPx4>, StHoldPosition4, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    SpiralPatternParams spiral;
    spiral.centerX = 0.0f;
    spiral.centerY = 0.0f;
    spiral.altitude = 20.0f;
    spiral.maxRadius = 15.0f;
    spiral.spacing = 3.0f;
    spiral.speed = 2.0f;
    configure_orthogonal<OrPx4, CbSpiralPattern>(spiral);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StSpiralPattern: starting search spiral...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StSpiralPattern: spiral search complete");
  }
};

}  // namespace sm_cl_px4_mr_test_2
