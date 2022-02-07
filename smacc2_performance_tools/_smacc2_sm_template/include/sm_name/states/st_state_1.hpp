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

namespace $sm_name$
{
// SMACC2 clases
using smacc2::EvStateRequestFinish;
using smacc2::Transition;

// STATE MACHINE SHARED VARIABLES (used in this state)
extern unsigned int _counter_;
extern rclcpp::Time _start_time_;

extern unsigned int _sum_of_iterations_;
extern double _sum_of_elapsed_time_;

// State constants
constexpr unsigned int ITERATIONS_CHECK = 1000;

// STATE DECLARATION
struct State1 : smacc2::SmaccState<State1, $SmName$>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<

    Transition<EvStateRequestFinish<State1>, State2>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  void onEntry()
  {
    // NOTE: counter is updated in 'State2'
    if (_counter_ == ITERATIONS_CHECK)
    {
      rclcpp::Duration elapsed = getNode()->now() - _start_time_;
      double frequency_Hz = ITERATIONS_CHECK / elapsed.seconds();
      _sum_of_iterations_ += ITERATIONS_CHECK;
      _sum_of_elapsed_time_ += elapsed.seconds();
      double global_frequency_Hz = _sum_of_iterations_ / _sum_of_elapsed_time_;

      // Using fatal to override all logging restrictions.
      RCLCPP_FATAL(
        getLogger(), "Executed %u iterations in %lf seconds: %lf Hz. Longtime frequency: %lf Hz",
        ITERATIONS_CHECK, elapsed.seconds(), frequency_Hz, global_frequency_Hz);

      _counter_ = 1;
      _start_time_ = getNode()->now();
    }

    this->postEvent<EvStateRequestFinish<State1>>();
  }

  void onExit() {}
};
}  // namespace $sm_name$
