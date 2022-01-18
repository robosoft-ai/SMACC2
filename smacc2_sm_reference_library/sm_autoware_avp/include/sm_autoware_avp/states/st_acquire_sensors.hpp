// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// CLIENTS
#include "ros_timer_client/cl_ros_timer.hpp"
#include "ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp"
#include "ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp"

// ORTHOGONALS
#include "sm_autoware_avp/orthogonals/or_autoware_auto.hpp"

namespace sm_autoware_avp
{
// SMACC2 clases
using smacc2::Transition;
using smacc2::EvStateRequestFinish;
using smacc2::default_transition_tags::SUCCESS;

// STATE DECLARATION
struct StAcquireSensors : smacc2::SmaccState<StAcquireSensors, SmAutowareAvp>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<

    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StAcquireSensors, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
    // configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);  // EvTimer triggers once at 10 client ticks
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}
