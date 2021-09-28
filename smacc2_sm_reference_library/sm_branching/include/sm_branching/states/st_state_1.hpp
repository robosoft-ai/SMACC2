// Copyright 2021 RobosoftAI Inc.
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

namespace sm_branching
{
using namespace cl_ros_timer;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc2::SmaccState<State1, SmBranching>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State2, SUCCESS>,
    Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, State2b, SUCCESS>,
    Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, State2c, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // EvTimer triggers each 3 client ticks
    configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);
    // EvTimer triggers once at 10 client ticks
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_branching
