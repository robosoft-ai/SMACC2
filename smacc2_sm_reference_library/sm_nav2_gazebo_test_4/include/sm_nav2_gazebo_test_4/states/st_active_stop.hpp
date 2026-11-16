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
// CbActiveStop: publishes zero velocity until the state is left; the timer leaves it.
struct StActiveStop : smacc2::SmaccState<StActiveStop, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StNavigateLastWaypoint, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateLastWaypoint, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbActiveStop>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(3s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};

}  // namespace sm_nav2_gazebo_test_4
