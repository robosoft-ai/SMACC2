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

#include <signal.h>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <lttng/tracepoint.h>
#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/smacc_signal_detector.hpp>
#include <smacc2/smacc_state_machine.hpp>
// #include <smacc2/smacc_tracing/trace_provider.hpp>
#include <smacc2/smacc_tracing/smacc_tracing.hpp>

//#include "tracetools/tracetools.h"

namespace smacc2
{
using namespace std::chrono_literals;
/**
******************************************************************************************************************
* SignalDetector()
******************************************************************************************************************
*/
SignalDetector::SignalDetector(SmaccFifoScheduler * scheduler, ExecutionModel executionModel)
{
  scheduler_ = scheduler;
  loop_rate_hz = 20.0;
  end_ = false;
  initialized_ = false;
  executionModel_ = executionModel;
}

rclcpp::Node::SharedPtr SignalDetector::getNode() { return this->smaccStateMachine_->getNode(); }

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void SignalDetector::initialize(ISmaccStateMachine * stateMachine)
{
  smaccStateMachine_ = stateMachine;
  lastState_ = std::numeric_limits<uint64_t>::quiet_NaN();
  findUpdatableClientsAndComponents();
  this->getNode()->declare_parameter("signal_detector_loop_freq", this->loop_rate_hz);

  initialized_ = true;
}

/**
******************************************************************************************************************
* findUpdatableClientsAndComponents()
******************************************************************************************************************
*/
void SignalDetector::findUpdatableClientsAndComponents()
{
  this->updatableClients_.clear();
  for (auto pair : this->smaccStateMachine_->getOrthogonals())
  {
    auto & orthogonal = pair.second;
    auto & clients = orthogonal->getClients();

    for (auto & client : clients)
    {
      // updatable client components
      auto updatableClient = dynamic_cast<ISmaccUpdatable *>(client.get());

      if (updatableClient != nullptr)
      {
        RCLCPP_DEBUG_STREAM(
          getLogger(), "Adding updatable client: " << demangleType(typeid(updatableClient)));
        this->updatableClients_.push_back(updatableClient);
      }

      // updatable client components
      std::vector<std::shared_ptr<ISmaccComponent>> components;
      client->getComponents(components);
      for (auto & componententry : components)
      {
        auto updatableComponent = dynamic_cast<ISmaccUpdatable *>(componententry.get());
        if (updatableComponent != nullptr)
        {
          RCLCPP_DEBUG_STREAM(
            getLogger(),
            "Adding updatable component: " << demangleType(typeid(*updatableComponent)));
          this->updatableClients_.push_back(updatableComponent);
        }
      }
    }
  }
}

/**
 ******************************************************************************************************************
 * findUpdatableClientBehaviors()
 ******************************************************************************************************************
 */
void SignalDetector::findUpdatableStateElements(ISmaccState * currentState)
{
  updatableStateElements_.push_back(std::vector<ISmaccUpdatable *>());

  auto & updatableElements = updatableStateElements_.back();

  for (auto pair : this->smaccStateMachine_->getOrthogonals())
  {
    auto & orthogonal = pair.second;
    auto & behaviors = orthogonal->getClientBehaviors().back();

    for (auto & currentBehavior : behaviors)
    {
      ISmaccUpdatable * updatableClientBehavior =
        dynamic_cast<ISmaccUpdatable *>(currentBehavior.get());

      if (updatableClientBehavior != nullptr)
      {
        RCLCPP_DEBUG_STREAM(
          getLogger(),
          "Adding updatable behavior: " << demangleType(typeid(updatableClientBehavior)));
        updatableElements.push_back(updatableClientBehavior);
      }
    }
  }

  auto updatableState = dynamic_cast<ISmaccUpdatable *>(currentState);
  if (updatableState != nullptr)
  {
    updatableElements.push_back(updatableState);
  }

  auto statereactors = currentState->getStateReactors();
  for (auto & sr : statereactors)
  {
    ISmaccUpdatable * updatableStateReactor = dynamic_cast<ISmaccUpdatable *>(sr.get());
    if (updatableStateReactor != nullptr)
    {
      RCLCPP_DEBUG_STREAM(
        getLogger(),
        "Adding updatable stateReactorr: " << demangleType(typeid(updatableStateReactor)));
      updatableElements.push_back(updatableStateReactor);
    }
  }

  auto eventgenerators = currentState->getEventGenerators();
  for (auto & eg : eventgenerators)
  {
    ISmaccUpdatable * updatableEventGenerator = dynamic_cast<ISmaccUpdatable *>(eg.get());
    if (updatableEventGenerator != nullptr)
    {
      RCLCPP_DEBUG_STREAM(
        getLogger(),
        "Adding updatable eventGenerator: " << demangleType(typeid(updatableEventGenerator)));
      updatableElements.push_back(updatableEventGenerator);
    }
  }
}

void SignalDetector::notifyStateConfigured(ISmaccState * currentState)
{
  this->findUpdatableStateElements(currentState);
}

void SignalDetector::notifyStateExited(ISmaccState * /*currentState*/)
{
  this->updatableStateElements_.pop_back();
}

/**
 ******************************************************************************************************************
 * setProcessorHandle()
 ******************************************************************************************************************
 */
void SignalDetector::setProcessorHandle(SmaccFifoScheduler::processor_handle processorHandle)
{
  processorHandle_ = processorHandle;
}

/**
 ******************************************************************************************************************
 * runThread()
 ******************************************************************************************************************
 */
void SignalDetector::runThread()
{
  signalDetectorThread_ = boost::thread(boost::bind(&SignalDetector::pollingLoop, this));
}

/**
 ******************************************************************************************************************
 * join()
 ******************************************************************************************************************
 */
void SignalDetector::join() { signalDetectorThread_.join(); }

/**
 ******************************************************************************************************************
 * stop()
 ******************************************************************************************************************
 */
void SignalDetector::stop() { end_ = true; }

/**
 ******************************************************************************************************************
 * poll()
 ******************************************************************************************************************
 */
void SignalDetector::pollOnce()
{
  // precondition: smaccStateMachine_ != nullptr

  //TRACEPOINT( spinOnce);
  TRACEPOINT(spinOnce);

  std::lock_guard<std::recursive_mutex> lock(smaccStateMachine_->m_mutex_);
  try
  {
    //smaccStateMachine_->lockStateMachine("update behaviors");

    this->findUpdatableClientsAndComponents();
    RCLCPP_DEBUG_STREAM(getLogger(), "updatable clients: " << this->updatableClients_.size());

    if (this->updatableClients_.size())
    {
      auto node = getNode();
      for (auto * updatableClient : this->updatableClients_)
      {
        auto updatableElementName = demangleType(typeid(*updatableClient)).c_str();
        try
        {
          RCLCPP_DEBUG_STREAM(
            node->get_logger(),
            "[PollOnce] update client call:  " << demangleType(typeid(*updatableClient)));

          TRACEPOINT(smacc2_state_update_start, updatableElementName);
          updatableClient->executeUpdate(smaccStateMachine_->getNode());
          TRACEPOINT(smacc2_state_update_start, updatableElementName);
        }
        catch (const std::exception & e)
        {
          RCLCPP_ERROR_STREAM(
            node->get_logger(),
            "Error in updatable elemnent " << updatableElementName << ": " << e.what() << '\n');
        }
      }
    }

    // STATE UPDATABLE ELEMENTS
    if (
      this->smaccStateMachine_->stateMachineCurrentAction !=
        StateMachineInternalAction::TRANSITIONING &&
      this->smaccStateMachine_->stateMachineCurrentAction !=
        StateMachineInternalAction::STATE_CONFIGURING &&
      this->smaccStateMachine_->stateMachineCurrentAction !=
        StateMachineInternalAction::STATE_ENTERING &&
      this->smaccStateMachine_->stateMachineCurrentAction !=
        StateMachineInternalAction::STATE_EXITING)
    {
      RCLCPP_DEBUG_STREAM(
        getLogger(), "updatable states: " << this->updatableStateElements_.size());

      for (auto stateElement : this->updatableStateElements_)
      {
        for (auto * udpatableStateElement : stateElement)
        {
          std::string updatableElementName = demangleType(typeid(*udpatableStateElement));
          auto updatableElementNameCstr = updatableElementName.c_str();

          RCLCPP_DEBUG_STREAM(
            getLogger(), "pollOnce update client behavior call: "
                           << demangleType(typeid(*udpatableStateElement)));
          TRACEPOINT(smacc2_state_update_start, updatableElementNameCstr);

          udpatableStateElement->executeUpdate(smaccStateMachine_->getNode());
          TRACEPOINT(smacc2_state_update_start, updatableElementNameCstr);
        }
      }
    }
  }
  catch (std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Exception during Signal Detector update loop. %s", ex.what());
  }

  auto nh = this->getNode();
  rclcpp::spin_some(nh);
  //smaccStateMachine_->unlockStateMachine("update behaviors");
}

/**
 ******************************************************************************************************************
 * pollingLoop()
 ******************************************************************************************************************
 */
void SignalDetector::pollingLoop()
{
  // rclcpp::Node::SharedPtr nh("~"); // use node name as root of the parameter server
  rclcpp::Node::SharedPtr _;
  rclcpp::Rate r0(20);

  while (!initialized_)
  {
    r0.sleep();
  }

  auto nh = getNode();

  if (!nh->get_parameter("signal_detector_loop_freq", this->loop_rate_hz))
  {
    RCLCPP_WARN(
      getLogger(),
      "Signal detector frequency (ros param signal_detector_loop_freq) was not set, using default "
      "frequency: "
      "%lf",
      this->loop_rate_hz);
  }
  else
  {
    RCLCPP_WARN(
      getLogger(), "Signal detector frequency (ros param signal_detector_loop_freq): %lf",
      this->loop_rate_hz);
  }

  nh->set_parameter(rclcpp::Parameter("signal_detector_loop_freq", this->loop_rate_hz));

  RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] loop rate hz:" << loop_rate_hz);

  if (this->executionModel_ == ExecutionModel::SINGLE_THREAD_SPINNER)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] running in single threaded mode");

    rclcpp::Rate r(loop_rate_hz);
    while (rclcpp::ok() && !end_)
    {
      RCLCPP_INFO_STREAM_THROTTLE(
        getLogger(), *getNode()->get_clock(), 10000, "[SignalDetector] heartbeat");
      pollOnce();
      rclcpp::spin_some(nh);
      r.sleep();
    }
  }
  else
  {
    RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] running in multi threaded mode");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();
  }
}

void onSigQuit(int)
{
  RCLCPP_INFO(rclcpp::get_logger("SMACC"), "SignalDetector: SIGQUIT received");
  exit(0);
}

}  // namespace smacc2
