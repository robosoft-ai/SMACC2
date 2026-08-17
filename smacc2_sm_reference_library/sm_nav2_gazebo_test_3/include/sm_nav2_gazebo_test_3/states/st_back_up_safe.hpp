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

namespace sm_nav2_gazebo_test_3
{

using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
//
// Collision-abort demo, step 3: back away from the wall - far enough to leave
// the inflated cost zone (0.7 m inflation radius), or the assisted-teleop
// guard that follows will zero every command from an in-collision-margin pose
struct StBackUpSafe : smacc2::SmaccState<StBackUpSafe, SmNav2GazeboTest3>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbBackUp, OrNavigation>, StAssistedTeleopGuard, SUCCESS>,
    Transition<EvCbFailure<CbBackUp, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAssistedTeleopGuard, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbBackUp>(1.2f, 0.1f);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StBackUpSafe: onEntry() - backing away from the wall");
  }
};

}  // namespace sm_nav2_gazebo_test_3
