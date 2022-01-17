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

//
// Author: Denis Štogl (template)
//

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// STATE DECLARATION
using namespace std::chrono;

namespace $sm_name$
{
// SMACC2 clases
using smacc2::EvStateRequestFinish;
using smacc2::Transition;

// STATE MACHINE SHARED VARIABLES (used in this state)
extern unsigned int _counter_;

// STATE DECLARATION
struct State2 : smacc2::SmaccState<State2, $SmName$>
{
  using SmaccState::SmaccState;
  // TRANSITION TABLE
  typedef boost::mpl::list<

    Transition<EvStateRequestFinish<State2>, State1>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  void onEntry()
  {
    // only update counter in this state
    ++_counter_;

    this->postEvent<EvStateRequestFinish<State2>>();
  }

  void onExit() {}
};
}  // namespace $sm_name$
