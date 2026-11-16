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
// CbAbortNavigation: cancels whatever navigation goal is in flight. It posts its success as soon as
// the cancel request is sent, and Nav2 aborts a goal that arrives while it is still cancelling the
// previous one, so the state is left by the timer and not by that event.
struct StAbortNavigation : smacc2::SmaccState<StAbortNavigation, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StRetryForward, SUCCESS>,
    Transition<EvCbFailure<CbAbortNavigation, OrNavigation>, StMissionFailed, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRetryForward, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbAbortNavigation>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(2s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};

}  // namespace sm_nav2_gazebo_test_4
