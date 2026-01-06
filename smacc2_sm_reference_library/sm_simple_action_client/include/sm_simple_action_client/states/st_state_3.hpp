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

#ifndef ROBOT_STATE_MACHINE__STATES__ST_STATE_3_HPP_
#define ROBOT_STATE_MACHINE__STATES__ST_STATE_3_HPP_

#include "smacc2/smacc.hpp"

namespace robot_state_machine
{
// STATE DECLARATION
struct StState3 : smacc2::SmaccState<StState3, SmSimpleActionClient>
{
  using SmaccState::SmaccState;

  // Declare custom object tags
  struct MANUAL_MODE : SUCCESS {};

  // TRANSITION TABLE
  typedef mpl::list<
    smacc2::Transition<EvManualMode<ClModeSelect, OrModeSelect>, StState1, MANUAL_MODE>
  > reactions;

  // State functions
  static void staticConfigure()
  {
    configure_orthogonal<OrModeSelect, CbModeSelect>();
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), " ");
    RCLCPP_INFO(getLogger(), "╔════════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(getLogger(), "║     FIBONACCI ACTION COMPLETED SUCCESSFULLY!                   ║");
    RCLCPP_INFO(getLogger(), "║                                                                ║");
    RCLCPP_INFO(getLogger(), "║  The state machine has reached StState3                        ║");
    RCLCPP_INFO(getLogger(), "║  Fibonacci calculation completed.                              ║");
    RCLCPP_INFO(getLogger(), "║                                                                ║");
    RCLCPP_INFO(getLogger(), "║  To return to StState1, publish:                               ║");
    RCLCPP_INFO(getLogger(), "║  ros2 topic pub /mode_command example_interfaces/msg/Int32 \\   ║");
    RCLCPP_INFO(getLogger(), "║      \"{data: 0}\" --once                                        ║");
    RCLCPP_INFO(getLogger(), "╚════════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(getLogger(), " ");
  }
};

}  // namespace robot_state_machine

#endif  // ROBOT_STATE_MACHINE__STATES__ST_STATE_3_HPP_
