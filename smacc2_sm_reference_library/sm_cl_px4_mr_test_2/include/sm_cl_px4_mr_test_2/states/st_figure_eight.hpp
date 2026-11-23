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

// STATE: Fly a figure-8 (lemniscate) pattern
struct StFigureEight : smacc2::SmaccState<StFigureEight, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFigureEight, OrPx4>, StHoldPosition6, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    FigureEightParams eight;
    eight.centerX = 5.0f;
    eight.centerY = 5.0f;
    eight.altitude = 20.0f;
    eight.size = 5.0f;
    eight.speed = 0.5f;
    eight.numLoops = 3;
    configure_orthogonal<OrPx4, CbFigureEight>(eight);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StFigureEight: starting figure-8 pattern...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StFigureEight: figure-8 complete");
  }
};

}  // namespace sm_cl_px4_mr_test_2
