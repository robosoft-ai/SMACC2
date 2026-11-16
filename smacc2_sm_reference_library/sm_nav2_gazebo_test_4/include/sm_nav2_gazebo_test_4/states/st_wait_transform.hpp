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
// CbWaitTransform: waits for the map to base_link transform with its own TF listener.
struct StWaitTransform : smacc2::SmaccState<StWaitTransform, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  struct NEXT : SUCCESS {};

  // clang-format off
  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitTransform, OrNavigation>, StLoadMissingFile, SUCCESS>,
    Transition<EvCbFailure<CbWaitTransform, OrNavigation>, StMissionFailed, ABORT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StLoadMissingFile, NEXT>
  > reactions;
  // clang-format on

  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbWaitTransform>("map", "base_link", rclcpp::Duration(10s));
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};

}  // namespace sm_nav2_gazebo_test_4
