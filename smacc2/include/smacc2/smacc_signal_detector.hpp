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

#include <atomic>
#include <boost/thread.hpp>
#include <smacc2/common.hpp>
#include <smacc2_msgs/msg/smacc_status.hpp>

namespace smacc2
{
enum class ExecutionModel
{
  SINGLE_THREAD_SPINNER,
  MULTI_THREAD_SPINNER
};

// Mostly define the state machine ros thread and receive events state machine components (clients, state elements)
// This class also contains the event queue of the state machine
class SignalDetector
{
public:
  SignalDetector(
    SmaccFifoScheduler * scheduler,
    ExecutionModel executionModel = ExecutionModel::SINGLE_THREAD_SPINNER);

  void initialize(ISmaccStateMachine * stateMachine);

  void setProcessorHandle(SmaccFifoScheduler::processor_handle processorHandle);

  // Runs the polling loop into a thread...
  void runThread();

  // Waits for the polling thread to end...
  void join();

  void stop();

  void pollingLoop();

  template <typename EventType>
  void postEvent(EventType * ev)
  {
    boost::intrusive_ptr<EventType> weakPtrEvent = ev;
    this->scheduler_->queue_event(processorHandle_, weakPtrEvent);
  }

  rclcpp::Node::SharedPtr getNode();
  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }

  void notifyStateConfigured(ISmaccState * currentState);

  void notifyStateExited(ISmaccState * currentState);

private:
  void pollOnce();

  ISmaccStateMachine * smaccStateMachine_;

  std::vector<ISmaccUpdatable *> updatableClients_;

  std::vector<std::vector<ISmaccUpdatable *>> updatableStateElements_;
  std::atomic<int64_t> lastState_;

  void findUpdatableClientsAndComponents();

  void findUpdatableStateElements(ISmaccState * currentState);

  // Loop frequency of the signal detector (to check answers from actionservers)
  double loop_rate_hz;

  std::atomic<bool> end_;

  std::atomic<bool> initialized_;

  rclcpp::Publisher<smacc2_msgs::msg::SmaccStatus>::SharedPtr statusPub_;

  // ---- boost statechart related ----

  SmaccFifoScheduler * scheduler_;

  SmaccFifoScheduler::processor_handle processorHandle_;

  boost::thread signalDetectorThread_;

  ExecutionModel executionModel_;
};

void onSigQuit(int sig);

// Main entry point for any SMACC state machine
// It instantiates and starts the specified state machine type
// it uses two threads: a new thread and the current one.
// The created thread is for the state machine process
// it locks the current thread to handle events of the state machine
template <typename StateMachineType>
void run(ExecutionModel executionModel = ExecutionModel::SINGLE_THREAD_SPINNER)
{
  ::signal(SIGQUIT, onSigQuit);

  // create the asynchronous state machine scheduler
  SmaccFifoScheduler scheduler1(true);

  // create the signalDetector component
  SignalDetector signalDetector(&scheduler1, executionModel);

  // create the asynchronous state machine processor
  SmaccFifoScheduler::processor_handle sm =
    scheduler1.create_processor<StateMachineType>(&signalDetector);

  // initialize the asynchronous state machine processor
  signalDetector.setProcessorHandle(sm);

  scheduler1.initiate_processor(sm);

  //create a thread for the asynchronous state machine processor execution
  boost::thread schedulerThread(boost::bind(&sc::fifo_scheduler<>::operator(), &scheduler1, 0));

  // use the  main thread for the signal detector component (waiting actionclient requests)
  signalDetector.pollingLoop();
}

struct SmExecution
{
  boost::thread * schedulerThread;
  boost::thread * signalDetectorLoop;
  SignalDetector * signalDetector;
  SmaccFifoScheduler * scheduler1;
  SmaccFifoScheduler::processor_handle sm;
};

template <typename StateMachineType>
SmExecution * run_async()
{
  ::signal(SIGQUIT, onSigQuit);

  SmExecution * ret = new SmExecution();

  // create the asynchronous state machine scheduler
  ret->scheduler1 = new SmaccFifoScheduler(true);

  // create the signalDetector component
  ret->signalDetector = new SignalDetector(ret->scheduler1);

  // create the asynchronous state machine processor
  ret->sm = ret->scheduler1->create_processor<StateMachineType>(ret->signalDetector);

  // initialize the asynchronous state machine processor
  ret->signalDetector->setProcessorHandle(ret->sm);

  ret->scheduler1->initiate_processor(ret->sm);

  //create a thread for the asynchronous state machine processor execution
  ret->schedulerThread =
    new boost::thread(boost::bind(&sc::fifo_scheduler<>::operator(), ret->scheduler1, NULL));
  ret->signalDetectorLoop =
    new boost::thread(boost::bind(&SignalDetector::pollingLoop, ret->signalDetector));

  return ret;
}

}  // namespace smacc2
