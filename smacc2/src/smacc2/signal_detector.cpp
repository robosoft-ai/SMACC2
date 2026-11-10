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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <signal.h>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <lttng/tracepoint.h>
#include <unistd.h>

#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/smacc_signal_detector.hpp>
#include <smacc2/smacc_state_machine.hpp>
#include <smacc2/smacc_tracing/smacc_tracing.hpp>

namespace smacc2
{
using namespace std::chrono_literals;

/**
******************************************************************************************************************
* SmExecution singleton implementation
******************************************************************************************************************
*/
SmExecution::SmExecution()
: schedulerThread(nullptr),
  signalDetectorLoop(nullptr),
  signalDetector(nullptr),
  scheduler1(nullptr)
{
}

SmExecution & SmExecution::getInstance()
{
  static SmExecution instance;
  return instance;
}

/**
******************************************************************************************************************
* SignalDetector()
******************************************************************************************************************
*/
SignalDetector::SignalDetector(SmaccFifoScheduler * scheduler, ExecutionModel executionModel)
{
  scheduler_ = scheduler;
  smaccStateMachine_ = nullptr;
  loop_rate_hz = 20.0;
  end_ = false;
  initialized_ = false;
  rosInitialized_ = false;
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
  lastState_ = std::numeric_limits<int64_t>::quiet_NaN();
  findUpdatableClientsAndComponents();
  this->getNode()->declare_parameter("signal_detector_loop_freq", this->loop_rate_hz);

  auto context = getNode()->get_node_base_interface()->get_context();
  context->on_shutdown(
    [this]()
    {
      this->stop();
      this->terminateScheduler();
    });

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

void SignalDetector::notifyRosInitialized()
{
  RCLCPP_INFO(getLogger(), "[SignalDetector] ROS initialization complete, enabling polling loop");
  rosInitialized_ = true;
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

void SignalDetector::joinAsyncClientBehaviorThreads()
{
  if (smaccStateMachine_ != nullptr)
  {
    smaccStateMachine_->joinAsyncClientBehaviorThreads();
  }
}

/**
 ******************************************************************************************************************
 * terminateScheduler()
 ******************************************************************************************************************
 */
void SignalDetector::terminateScheduler()
{
  if (scheduler_)
  {
    RCLCPP_INFO(getLogger(), "[SignalDetector] Terminating scheduler...");
    scheduler_->terminate();
  }
}

/**
 ******************************************************************************************************************
 * poll()
 ******************************************************************************************************************
 */
void SignalDetector::pollOnce()
{
  // precondition: smaccStateMachine_ != nullptr

  TRACETOOLS_TRACEPOINT(spinOnce);

  std::lock_guard<std::recursive_mutex> lock(smaccStateMachine_->m_mutex_);
  try
  {
    this->findUpdatableClientsAndComponents();
    RCLCPP_DEBUG_STREAM(getLogger(), "Updatable clients: " << this->updatableClients_.size());

    if (this->updatableClients_.size())
    {
      auto node = getNode();
      for (auto * updatableClient : this->updatableClients_)
      {
        auto updatableElementNameStr = demangleType(typeid(*updatableClient));
        auto updatableElementName = updatableElementNameStr.c_str();
        try
        {
          RCLCPP_DEBUG_STREAM(
            node->get_logger(),
            "[PollOnce] update client call:  " << demangleType(typeid(*updatableClient)));

          TRACETOOLS_TRACEPOINT(smacc2_state_update_start, updatableElementName);
          updatableClient->executeUpdate(smaccStateMachine_->getNode());
          TRACETOOLS_TRACEPOINT(smacc2_state_update_start, updatableElementName);
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
    // One load, one decision: the state machine thread can change the phase between two
    // reads, so the gate must be evaluated on a single snapshot.
    const StateMachineInternalAction currentAction =
      this->smaccStateMachine_->stateMachineCurrentAction.load();
    if (
      currentAction != StateMachineInternalAction::TRANSITIONING &&
      currentAction != StateMachineInternalAction::STATE_CONFIGURING &&
      currentAction != StateMachineInternalAction::STATE_ENTERING &&
      currentAction != StateMachineInternalAction::STATE_EXITING &&
      currentAction != StateMachineInternalAction::STATE_MACHINE_INITIALIZING)
    {
      RCLCPP_DEBUG_STREAM(
        getLogger(), "Updatable states: " << this->updatableStateElements_.size());

      for (auto stateElement : this->updatableStateElements_)
      {
        for (auto * udpatableStateElement : stateElement)
        {
          std::string updatableElementName = demangleType(typeid(*udpatableStateElement));
          auto updatableElementNameCstr = updatableElementName.c_str();

          RCLCPP_DEBUG_STREAM(
            getLogger(), "pollOnce update client behavior call: "
                           << demangleType(typeid(*udpatableStateElement)));
          TRACETOOLS_TRACEPOINT(smacc2_state_update_start, updatableElementNameCstr);

          udpatableStateElement->executeUpdate(smaccStateMachine_->getNode());
          TRACETOOLS_TRACEPOINT(smacc2_state_update_start, updatableElementNameCstr);
        }
      }
    }
  }
  catch (std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Exception during Signal Detector update loop. %s.", ex.what());
  }
}

// rclcpp::Rate::sleep() goes through rclcpp::Clock::sleep_until, which throws std::runtime_error
// when the context was shut down between the caller's rclcpp::ok() check and the sleep. Shutdown
// runs on rclcpp's signal thread, concurrently with the polling loop, so the window is everything
// an iteration does: pollOnce (including waiting for m_mutex_ during a transition and every
// update()), the callback drain, and the Rate arithmetic. At shutdown the throw is the expected
// end of the loop, not an error; anything else is rethrown. Returns false once the context is
// invalid.
static bool sleepUnlessShutdown(rclcpp::Rate & rate)
{
  try
  {
    rate.sleep();
  }
  catch (const std::runtime_error &)
  {
    if (rclcpp::ok())
    {
      throw;
    }
    return false;
  }
  return true;
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

  // Wait for both SignalDetector::initialize() (called from ISmaccStateMachine constructor)
  // and ROS initialization to complete (initializeROS called from initiate_impl).
  // This ensures orthogonals, clients, and ROS objects are fully initialized
  // before we start polling and accessing them.
  while ((!initialized_ || !rosInitialized_) && rclcpp::ok() && !end_)
  {
    if (!sleepUnlessShutdown(r0))
    {
      break;
    }
  }

  if (!rclcpp::ok() || end_)
  {
    RCLCPP_INFO(getLogger(), "[SignalDetector] Shutdown requested before initialization completed");
    return;
  }

  auto nh = getNode();

  if (!nh->get_parameter("signal_detector_loop_freq", this->loop_rate_hz))
  {
    RCLCPP_WARN(
      getLogger(),
      "Signal Detector frequency (ros param signal_detector_loop_freq) was not set, using default "
      "frequency: "
      "%lf",
      this->loop_rate_hz);
  }
  else
  {
    RCLCPP_WARN(
      getLogger(), "Signal Detector frequency (ros param signal_detector_loop_freq): %lf",
      this->loop_rate_hz);
  }

  // A rate of zero or less has no period, and spin_all(0) below spins forever: use the default.
  if (!(this->loop_rate_hz > 0.0))
  {
    RCLCPP_ERROR(
      getLogger(), "Signal Detector frequency %lf is not positive, using the default 20 Hz",
      this->loop_rate_hz);
    this->loop_rate_hz = 20.0;
  }

  nh->set_parameter(rclcpp::Parameter("signal_detector_loop_freq", this->loop_rate_hz));

  RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] Loop rate hz:" << loop_rate_hz);

  if (this->executionModel_ == ExecutionModel::SINGLE_THREAD_SPINNER)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] Running in single-threaded mode.");

    // One executor for the whole loop. rclcpp::spin_some(nh) built a new executor on every
    // iteration, and an executor constructor creates a guard condition, which throws
    // RCLError if rclcpp::shutdown() lands between the loop condition and that call.
    // Shutdown now runs on rclcpp's signal thread, concurrently with this loop, so the loop
    // must tolerate it: an existing executor's spin_all() simply returns once the context
    // is invalid.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    rclcpp::Rate r(loop_rate_hz);
    // Drain every ready callback each iteration, as ROS 1's spinOnce did, bounded by one loop
    // period. spin_some() took one callback per entity per iteration, sampling faster topics.
    const auto spinBudget = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / loop_rate_hz));
    const double spinBudgetMs = std::chrono::duration<double, std::milli>(spinBudget).count();
    while (rclcpp::ok() && !end_)
    {
      RCLCPP_INFO_STREAM_THROTTLE(
        getLogger(), *getNode()->get_clock(), 10000, "[SignalDetector] Heartbeat");
      pollOnce();
      const auto spinStart = std::chrono::steady_clock::now();
      executor.spin_all(spinBudget);
      if (std::chrono::steady_clock::now() - spinStart >= spinBudget)
      {
        RCLCPP_WARN_THROTTLE(
          getLogger(), *getNode()->get_clock(), 5000,
          "[SignalDetector] callbacks filled the whole loop period (%.1f ms): the loop is "
          "saturated and runs below %.1f Hz",
          spinBudgetMs, loop_rate_hz);
      }
      if (!sleepUnlessShutdown(r))
      {
        break;
      }
    }
    executor.remove_node(nh);
  }
  else
  {
    RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] Running in multi-threaded mode.");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();
  }
}

// Hard escape hatch. Only async-signal-safe calls: no logging (takes a mutex), no exit()
// (runs static destructors and atexit handlers in signal context).
void onSigQuit(int)
{
  static const char msg[] = "SMACC: SIGQUIT received, exiting immediately\n";
  ssize_t unused = ::write(STDERR_FILENO, msg, sizeof(msg) - 1);
  (void)unused;
  ::_exit(128 + SIGQUIT);
}

}  // namespace smacc2
