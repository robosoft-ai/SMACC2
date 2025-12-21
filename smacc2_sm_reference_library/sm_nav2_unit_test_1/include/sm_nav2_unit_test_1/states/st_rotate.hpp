// Copyright 2024 RobosoftAI Inc.
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

namespace sm_nav2_unit_test_1
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct StRotate : smacc2::SmaccState<StRotate, SmNav2UnitTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StNavigateToWaypoint2, SUCCESS>,
    Transition<EvCbFailure<CbPureSpinning, OrNavigation>, StFinalState, ABORT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Rotate 180 degrees (PI radians) at 0.5 rad/s
    configure_orthogonal<OrNavigation, CbPureSpinning>(M_PI, 0.5);

    // Keyboard behavior for manual control
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StRotate: runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StRotate: onEntry() - Rotating 180 degrees");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StRotate: onExit()");
  }
};

}  // namespace sm_nav2_unit_test_1
