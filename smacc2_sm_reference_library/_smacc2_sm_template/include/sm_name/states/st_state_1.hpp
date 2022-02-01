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
using $sm_name$::OrTimer;  // This is example variable - feel free to delete it.

namespace $sm_name$
{
// SMACC2 clases
using smacc2::Transition;
using smacc2::EvStateRequestFinish;
using smacc2::default_transition_tags::SUCCESS;

using cl_ros_timer::EvTimer;
using cl_ros_timer::CbTimerCountdownLoop;
using cl_ros_timer::CbTimerCountdownOnce;

using $sm_name$::OrTimer;

// STATE MACHINE SHARED VARIABLES (used in this state)
extern std::shared_ptr<std::string> _output_message_prefix_;    // This is example variable - feel free to delete it.

// STATE DECLARATION
struct State1 : smacc2::SmaccState<State1, $SmName$>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE - adjust as needed
  typedef boost::mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State2, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // START: Example code - change needed
    configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);  // EvTimer triggers once at 10 client ticks
    // END: Example code - change or delete as needed
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    // START: Example code - change or delete as needed
    RCLCPP_INFO(getLogger(), (*_output_message_prefix_ + " On Entry!").c_str());
    // END: Example code - change or delete as needed
  }

  void onExit()
  {
    // START: Example code - change or delete as needed
    RCLCPP_INFO(getLogger(), (*_output_message_prefix_ + " On Exit!").c_str());
    // END: Example code - change or delete as needed
  }
};
}
