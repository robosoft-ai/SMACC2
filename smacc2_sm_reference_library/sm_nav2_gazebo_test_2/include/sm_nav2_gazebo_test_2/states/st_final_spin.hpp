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

namespace sm_nav2_gazebo_test_2
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
//
// Mission complete: one full in-place 360 degree rotation before the final state
struct StFinalSpin : smacc2::SmaccState<StFinalSpin, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StFinalState, SUCCESS>,
    Transition<EvCbFailure<CbPureSpinning, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StFinalState, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Full 360 degree rotation at 0.5 rad/s
    configure_orthogonal<OrNavigation, CbPureSpinning>(2 * M_PI, 0.5);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StFinalSpin: onEntry() - Mission complete, victory rotation");
  }
};

}  // namespace sm_nav2_gazebo_test_2
