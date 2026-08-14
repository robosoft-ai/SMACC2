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
// Navigates from the spawn area to (-2.0, 2.5). The curve_pillar at (-2.0, 1.0)
// sits directly on the straight line, so the planned and driven trajectory must
// bow around it - producing a genuinely CURVED recorded path for StUndoCurve to
// retrace. CbNavigateGlobalPosition records the path with the odom tracker.
struct StNavigateWithCurve : smacc2::SmaccState<StNavigateWithCurve, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  // Behavior events are state-scoped (cannot go stale across transitions),
  // unlike the machine-scoped EvActionSucceeded component events
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StUndoCurve, SUCCESS>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StUndoCurve, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Curve leg goal: north of the pillar, final heading 90 degrees
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-2.0, 2.5, M_PI_2);

    // Keyboard behavior for manual control
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "StNavigateWithCurve: onEntry() - Navigating to (-2.0, 2.5) around the pillar "
      "(recording curved path)");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StNavigateWithCurve: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_2
