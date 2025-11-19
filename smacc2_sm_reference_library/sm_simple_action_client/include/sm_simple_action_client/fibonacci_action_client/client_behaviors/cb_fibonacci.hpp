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

#ifndef ROBOT_STATE_MACHINE__CLIENTS__CB_FIBONACCI_HPP_
#define ROBOT_STATE_MACHINE__CLIENTS__CB_FIBONACCI_HPP_

#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include "sm_simple_action_client/fibonacci_action_client/cl_fibonacci.hpp"

namespace robot_state_machine
{
class CbFibonacci : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbFibonacci() {}
  // virtual ~CbFibonacci();

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresClient(cl_fibonacci_);
    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  void onEntry() override
  {
    ClFibonacci::Goal goal;
    goal.order = 10;

    fibonacciCallback_ = std::make_shared<ClFibonacci::FibonacciResultSignal>();
    this->getStateMachine()->createSignalConnection(
      *fibonacciCallback_, &CbFibonacci::onFibonacciResult, this);
    goalHandleFuture_ = cl_fibonacci_->sendGoal(goal, fibonacciCallback_);
  }
  void onExit() override { cl_fibonacci_->cancelGoal(); }

private:
  void onFibonacciResult(const ClFibonacci::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      onFibonacciActionSuccess();
    else
      onFibonacciActionAbort();
  }
  void onFibonacciActionSuccess() { this->postSuccessEvent(); }
  void onFibonacciActionAbort() { this->postFailureEvent(); }

  ClFibonacci * cl_fibonacci_;
  ClFibonacci::FibonacciResultSignal::SharedPtr fibonacciCallback_;
  rclcpp_action::ResultCode fibonacciResult_;
  std::shared_future<std::shared_ptr<
    rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::Fibonacci> > >
    goalHandleFuture_;
};
}  // namespace robot_state_machine

#endif  // ROBOT_STATE_MACHINE__CLIENTS__CB_FIBONACCI_HPP_
