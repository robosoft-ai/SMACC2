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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc2/common.hpp>

#include <smacc2/smacc_state_base.hpp>
#include <smacc2/smacc_state_machine.hpp>

namespace smacc2
{
/// State Machine
template <typename DerivedStateMachine, typename InitialStateType>
struct SmaccStateMachineBase
: public ISmaccStateMachine,
  public sc::asynchronous_state_machine<
    DerivedStateMachine, InitialStateType, SmaccFifoScheduler, SmaccAllocator>
{
public:
  SmaccStateMachineBase(my_context ctx, SignalDetector * signalDetector)
  : ISmaccStateMachine(smacc2::utils::cleanShortTypeName<DerivedStateMachine>(), signalDetector),
    sc::asynchronous_state_machine<
      DerivedStateMachine, InitialStateType, SmaccFifoScheduler, SmaccAllocator>(ctx)
  {
  }

  virtual ~SmaccStateMachineBase()
  {
    //updateCurrentState<InitialStateType>(false);
  }

  void reset() override
  {
    ISmaccStateMachine::reset();
    this->terminate();
    smacc2::run<DerivedStateMachine>();
  }

  void stop() override
  {
    ISmaccStateMachine::stop();
    this->terminate();
  }

  void eStop() override
  {
    ISmaccStateMachine::eStop();
    this->terminate();
  }

  void initiate_impl() override
  {
    globalNh_ = this->getNode();

    // this is before because this creates orthogonals
    this->onInitialize();

    RCLCPP_INFO(getLogger(), "[SmaccStateMachine] Introspecting state machine via typeWalker");
    this->buildStateMachineInfo<InitialStateType>();

    RCLCPP_INFO(getLogger(), "[SmaccStateMachine] initiate_impl");
    auto shortname = smacc2::utils::cleanShortTypeName(typeid(DerivedStateMachine));

    this->initializeROS(shortname);

    RCLCPP_INFO(getLogger(), "[SmaccStateMachine] Initializing ROS communication mechanisms");
    this->onInitialized();

    // publish startup state machine transition info
    auto transitionInfo = std::make_shared<SmaccTransitionInfo>();
    transitionInfo->destinyState = this->stateMachineInfo_->getState<InitialStateType>();
    this->publishTransition(*transitionInfo);

    RCLCPP_INFO(getLogger(), "[SmaccStateMachine] Initializing state machine");
    sc::state_machine<DerivedStateMachine, InitialStateType, SmaccAllocator>::initiate();
  }
};
}  // namespace smacc2
