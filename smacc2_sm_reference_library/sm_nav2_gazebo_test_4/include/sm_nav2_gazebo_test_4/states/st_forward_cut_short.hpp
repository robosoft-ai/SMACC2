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

namespace sm_nav2_gazebo_test_4
{
// A 2 m forward motion that the timer cuts short after 3 s, to leave a goal for the retry.
struct StForwardCutShort : smacc2::SmaccState<StForwardCutShort, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StAbortNavigation, SUCCESS>,
    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StAbortNavigation, SUCCESS>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StMissionFailed, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAbortNavigation, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(2.0f);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(3s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};

}  // namespace sm_nav2_gazebo_test_4
