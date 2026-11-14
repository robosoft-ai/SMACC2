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
#pragma once

#include <atomic>
#include <boost/any.hpp>
#include <deque>
#include <map>
#include <mutex>
#include <type_traits>

#include <smacc2/common.hpp>
#include <smacc2/introspection/introspection.hpp>
#include <smacc2/introspection/smacc_state_machine_info.hpp>
#include <smacc2/smacc_signal.hpp>
#include <smacc2/smacc_updatable.hpp>

#include <smacc2_msgs/msg/smacc_state_machine.hpp>
#include <smacc2_msgs/msg/smacc_status.hpp>
#include <smacc2_msgs/msg/smacc_transition_log_entry.hpp>
#include <smacc2_msgs/srv/smacc_get_transition_history.hpp>

#include <smacc2/callback_counter_semaphore.hpp>
#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_reactor.hpp>

namespace smacc2
{
using namespace smacc2::introspection;

enum class EventLifeTime
{
  ABSOLUTE,
  CURRENT_STATE /*events are discarded if we are leaving the state it were created. I is used for client behaviors whose lifetime is associated to state*/
};

enum class StateMachineInternalAction
{
  STATE_CONFIGURING,
  STATE_ENTERING,
  STATE_RUNNING,
  STATE_EXITING,
  TRANSITIONING,
  STATE_MACHINE_INITIALIZING  // before the initial state is constructed
};

// This class describes the concept of Smacc State Machine in an abastract way.
// The SmaccStateMachineBase inherits from this state machine and from
// statechart::StateMachine<> (via multiple inheritance)
class ISmaccStateMachine
{
public:
  ISmaccStateMachine(
    std::string stateMachineName, SignalDetector * signalDetector,
    rclcpp::NodeOptions nodeOptions = rclcpp::NodeOptions());

  virtual ~ISmaccStateMachine();

  // Not copyable or movable. This turns an accidental copy, such as a missing & in
  // "auto & sm = *stateMachine;", into a compile error instead of calls on a detached copy.
  ISmaccStateMachine(const ISmaccStateMachine &) = delete;
  ISmaccStateMachine & operator=(const ISmaccStateMachine &) = delete;
  ISmaccStateMachine(ISmaccStateMachine &&) = delete;
  ISmaccStateMachine & operator=(ISmaccStateMachine &&) = delete;

  virtual void reset();

  virtual void stop();

  virtual void eStop();

  // Asks every async client behavior of the active state(s) to finish and joins its onEntry
  // thread. Called from run<> after the scheduler thread has joined and before this object
  // is destroyed; ~state_machine destroys the active states without exit actions, so nothing
  // else stops those threads at shutdown. Runs no onExit code.
  void joinAsyncClientBehaviorThreads();

  template <typename TOrthogonal>
  TOrthogonal * getOrthogonal();

  // gets the client behavior in a given orthogonal
  // the index is used to distinguish between multiple client behaviors of the same type
  template <typename TOrthogonal, typename TClientBehavior>
  inline TClientBehavior * getClientBehavior(int index = 0)
  {
    auto orthogonal = this->template getOrthogonal<TOrthogonal>();

    return orthogonal->template getClientBehavior<TClientBehavior>(index);
  }

  const std::map<std::string, std::shared_ptr<smacc2::ISmaccOrthogonal>> & getOrthogonals() const;

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage, ComponentRequirement requirementType);

  template <typename SmaccComponentType>
  void requiresComponent(
    const std::string & name, SmaccComponentType *& storage, ComponentRequirement requirementType);

  // Takes ownership of `ev` (must be allocated with `new`); the caller must not use it after
  // this call. Released by the scheduler after dispatch, or here if discarded or on exception.
  template <typename EventType>
  void postEvent(EventType * ev, EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);

  template <typename EventType>
  void postEvent(EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);

  // gets data from the state machine blackboard
  template <typename T>
  bool getGlobalSMData(const std::string & name, T & ret);

  // sets data in the state machine blackboard
  template <typename T>
  void setGlobalSMData(const std::string & name, T value);

  std::string getStateMachineName();

  void state_machine_visualization();

  inline std::shared_ptr<SmaccStateInfo> getCurrentStateInfo() { return currentStateInfo_; }

  void publishTransition(const SmaccTransitionInfo & transitionInfo);

  /// this function should be implemented by the user to create the orthogonals
  virtual void onInitialize();

  // Called when a state's runtimeConfigure or onEntry threw and no state, up to the outermost
  // container, reacted to EvStInitializationFailure. The default logs; the machine stays in that state.
  virtual void onUnhandledStInitializationFailure(
    const std::string & stateName, const std::string & what);

  void getTransitionLogHistory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<smacc2_msgs::srv::SmaccGetTransitionHistory::Request> req,
    std::shared_ptr<smacc2_msgs::srv::SmaccGetTransitionHistory::Response> res);

  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  smacc2::SmaccSignalConnection createSignalConnection(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object);

  // Finalizes and erases the signal connections registered by a state-scoped object.
  // The typed overload normalizes the key with dynamic_cast<void *> (the complete object's
  // address), as createSignalConnection does, so a base-subobject pointer and the
  // most-derived pointer of the same object always map to the same entry.
  void disconnectSmaccSignalObject(void * object_ptr);

  template <typename T>
  std::enable_if_t<std::is_polymorphic<T>::value> disconnectSmaccSignalObject(T * object)
  {
    disconnectSmaccSignalObject(dynamic_cast<void *>(object));
  }

  template <typename StateType>
  void notifyOnStateEntryStart(StateType * state);

  template <typename StateType>
  void notifyOnStateEntryEnd(StateType * state);

  template <typename StateType>
  void notifyOnRuntimeConfigured(StateType * state);

  template <typename StateType>
  void notifyOnStateExiting(StateType * state);

  template <typename StateType>
  void notifyOnStateExited(StateType * state);

  template <typename StateType>
  void notifyOnRuntimeConfigurationFinished(StateType * state);

  inline int64_t getCurrentStateCounter() const;

  inline ISmaccState * getCurrentState() const;

  inline const SmaccStateMachineInfo & getStateMachineInfo();

  template <typename InitialStateType>
  void buildStateMachineInfo();

  rclcpp::Node::SharedPtr getNode();

  inline rclcpp::Logger getLogger() { return nh_->get_logger(); }

  inline std::recursive_mutex & getMutex() { return this->m_mutex_; }

protected:
  void checkStateMachineConsistence();

  void initializeROS(const std::string & shortname);

  void onInitialized();

  template <typename TOrthogonal>
  void createOrthogonal();

  // The node handle for this state
  rclcpp::Node::SharedPtr nh_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccStateMachine>::SharedPtr stateMachinePub_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccStatus>::SharedPtr stateMachineStatusPub_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccTransitionLogEntry>::SharedPtr transitionLogPub_;

  rclcpp::Publisher<smacc2_msgs::msg::SmaccEvent>::SharedPtr eventsLogPub_;

  rclcpp::Service<smacc2_msgs::srv::SmaccGetTransitionHistory>::SharedPtr transitionHistoryService_;

  // if it is null, you may be located in a transition. There is a small gap of time where internally
  // this currentState_ is null. This may change in the future.
  std::vector<ISmaccState *> currentState_;

  std::shared_ptr<SmaccStateInfo> currentStateInfo_;

  smacc2_msgs::msg::SmaccStatus status_msg_;

  // orthogonals
  std::map<std::string, std::shared_ptr<smacc2::ISmaccOrthogonal>> orthogonals_;

protected:
  std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo_;

private:
  std::recursive_mutex m_mutex_;
  std::recursive_mutex eventQueueMutex_;

  // Written on the state machine thread (notifyOnState* / notifyOnRuntimeConfigured), read
  // by SignalDetector::pollOnce on the polling thread and by postEvent on any thread.
  // Atomic so every reader sees the latest phase; initialized so pollOnce has a defined
  // value before the first state exists.
  std::atomic<StateMachineInternalAction> stateMachineCurrentAction{
    StateMachineInternalAction::STATE_MACHINE_INITIALIZING};

  std::map<void *, std::shared_ptr<CallbackCounterSemaphore>> stateCallbackConnections;

  // shared variables
  std::map<std::string, std::pair<std::function<std::string()>, boost::any>> globalData_;

  // contains the whole history of transitions of the state machine
  // The last transition_log_history_size transitions, for the history service. Pushed on the state
  // machine thread, copied on the executor thread; its own leaf mutex, never m_mutex_ (X).
  std::deque<smacc2_msgs::msg::SmaccTransitionLogEntry> transitionLogHistory_;
  std::mutex transitionLogMutex_;
  size_t transitionLogHistorySize_ = 1000;

  smacc2::SMRunMode runMode_;

  // Event to notify to the signaldetection thread that a request has been created...
  SignalDetector * signalDetector_;

  int64_t stateSeqCounter_ = 0;

  void lockStateMachine(const std::string & msg);

  void unlockStateMachine(const std::string & msg);

  template <typename EventType>
  void propagateEventToStateReactors(ISmaccState * st, EventType * ev);

  // Depth of the reactor walk on the calling thread; nonzero only inside the walk work item.
  static thread_local int reactorWalkDepth_;

  void updateStatusMessage();

  friend class ISmaccState;
  friend class SignalDetector;
};
}  // namespace smacc2

#include <smacc2/impl/smacc_client_impl.hpp>
#include <smacc2/impl/smacc_component_impl.hpp>
#include <smacc2/impl/smacc_orthogonal_impl.hpp>
#include <smacc2/impl/smacc_state_impl.hpp>
