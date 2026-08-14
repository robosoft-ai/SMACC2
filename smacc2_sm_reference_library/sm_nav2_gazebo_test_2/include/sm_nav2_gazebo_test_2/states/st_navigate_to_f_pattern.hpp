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
// Moves to the F pattern start corner: rays run east from here, rows pitch
// north (free space verified against walls and both pillars).
struct StNavigateToFPattern : smacc2::SmaccState<StNavigateToFPattern, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS2::SsFPattern1, SUCCESS>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, SS2::SsFPattern1, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // F pattern start: x=0.0, y=-1.5, facing east (ray direction)
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.0, -1.5, 0.0);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StNavigateToFPattern: onEntry() - Navigating to F pattern start (0.0, -1.5)");
  }
};

}  // namespace sm_nav2_gazebo_test_2
