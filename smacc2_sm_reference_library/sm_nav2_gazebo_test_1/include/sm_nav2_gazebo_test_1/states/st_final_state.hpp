// Copyright 2024 RobosoftAI Inc.
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

namespace sm_nav2_gazebo_test_1
{

// STATE DECLARATION
struct StFinalState : smacc2::SmaccState<StFinalState, SmNav2GazeboTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE - Terminal state, no transitions
  typedef mpl::list<> reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // No behaviors configured - terminal state
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StFinalState: runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StFinalState: onEntry() - Mission Complete!");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StFinalState: onExit()");
  }
};

}  // namespace sm_nav2_gazebo_test_1
