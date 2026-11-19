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

#pragma once

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "smacc2/client_bases/smacc_action_client_base.hpp"
#include "smacc2/smacc.hpp"

namespace robot_state_machine
{
class ClFibonacci
: public smacc2::client_bases::SmaccActionClientBase<action_tutorials_interfaces::action::Fibonacci>
{
public:
  using smacc2::client_bases::SmaccActionClientBase<
    action_tutorials_interfaces::action::Fibonacci>::GoalHandle;
  using smacc2::client_bases::SmaccActionClientBase<
    action_tutorials_interfaces::action::Fibonacci>::ResultCallback;

  typedef smacc2::SmaccSignal<void(const WrappedResult &)> FibonacciResultSignal;

  ClFibonacci(std::string action_name = "/fibonacci")
  : smacc2::client_bases::SmaccActionClientBase<action_tutorials_interfaces::action::Fibonacci>(
      std::move(action_name))
  {
  }

  virtual ~ClFibonacci() {}
};
}  // namespace robot_state_machine
