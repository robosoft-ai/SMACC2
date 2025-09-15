// Copyright 2025 Robosoft Inc.
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

#include <smacc2/smacc.hpp>

namespace sm_cl_keyboard_unit_test_1
{
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace cl_keyboard::components;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct State2 : smacc2::SmaccState<State2, SmClKeyboardUnitTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State1, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, State1, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);  // EvTimer triggers once at 10 client ticks
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "runtimeConfigure() firing"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "onEntry() firing"); }

  void onExit() { RCLCPP_INFO(getLogger(), "onExit() firing"); }
};
}  // namespace sm_cl_keyboard_unit_test_1
