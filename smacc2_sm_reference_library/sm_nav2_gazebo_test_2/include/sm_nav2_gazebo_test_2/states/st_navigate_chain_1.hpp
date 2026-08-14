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
// First leg of the chained-undo phase: curve around the first pillar to
// P1 (-2.0, 2.5). The odom tracker pushes the previous trail onto its stack and
// records this leg; the recording survives until StUndoChain1 retraces it.
struct StNavigateChain1 : smacc2::SmaccState<StNavigateChain1, SmNav2GazeboTest2>
{
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct NEXT : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StNavigateChain2, SUCCESS>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StFinalState, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateChain2, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Chain leg 1: same curved leg as the single-undo phase
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-2.0, 2.5, M_PI_2);

    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StNavigateChain1: onEntry() - chained-undo phase, leg 1 to P1 (-2.0, 2.5)");
  }
};

}  // namespace sm_nav2_gazebo_test_2
