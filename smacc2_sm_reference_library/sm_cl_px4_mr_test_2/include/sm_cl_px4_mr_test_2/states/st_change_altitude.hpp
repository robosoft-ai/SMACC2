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

// STATE: Climb to 20m altitude
struct StChangeAltitude : smacc2::SmaccState<StChangeAltitude, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbChangeAltitude, OrPx4>, StHoldPosition3, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Ascend to 20 meters (positive = meters above ground)
    configure_orthogonal<OrPx4, CbChangeAltitude>(20.0f);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StChangeAltitude: climbing to 20m...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StChangeAltitude: altitude reached");
  }
};

}  // namespace sm_cl_px4_mr_test_2
