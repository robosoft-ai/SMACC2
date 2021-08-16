/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/common.h>

#include <smacc/smacc_state_base.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
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
  : ISmaccStateMachine(smacc::utils::cleanShortTypeName<DerivedStateMachine>(), signalDetector),
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
    smacc::run<DerivedStateMachine>();
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

    RCLCPP_INFO(
      getNode()->get_logger(), "[SmaccStateMachine] Introspecting state machine via typeWalker");
    this->buildStateMachineInfo<InitialStateType>();

    RCLCPP_INFO(getNode()->get_logger(), "[SmaccStateMachine] initiate_impl");
    auto shortname = smacc::utils::cleanShortTypeName(typeid(DerivedStateMachine));

    this->initializeROS(shortname);

    RCLCPP_INFO(
      getNode()->get_logger(), "[SmaccStateMachine] Initializing ROS communication mechanisms");
    this->onInitialized();

    RCLCPP_INFO(getNode()->get_logger(), "[SmaccStateMachine] Initializing state machine");
    sc::state_machine<DerivedStateMachine, InitialStateType, SmaccAllocator>::initiate();
  }
};
}  // namespace smacc
