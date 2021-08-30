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

#include <boost/any.hpp>
#include <map>
#include <mutex>

#include <smacc2/common.hpp>
#include <smacc2/introspection/introspection.hpp>
#include <smacc2/introspection/smacc_state_machine_info.hpp>
#include <smacc2/smacc_signal.hpp>
#include <smacc2/smacc_updatable.hpp>

#include <smacc2_msgs/msg/smacc_state_machine.hpp>
#include <smacc2_msgs/msg/smacc_status.hpp>
#include <smacc2_msgs/msg/smacc_transition_log_entry.hpp>
#include <smacc2_msgs/srv/smacc_get_transition_history.hpp>

#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_reactor.hpp>
//#include <smacc2/smacc_event_generator.hpp>

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
  STATE_STEADY,
  STATE_EXITING,
  TRANSITIONING
};

// This class describes the concept of Smacc State Machine in an abastract way.
// The SmaccStateMachineBase inherits from this state machine and from
// statechart::StateMachine<> (via multiple inheritance)
class ISmaccStateMachine
{
public:
  ISmaccStateMachine(std::string stateMachineName, SignalDetector * signalDetector);

  virtual ~ISmaccStateMachine();

  virtual void reset();

  virtual void stop();

  virtual void eStop();

  template <typename TOrthogonal>
  TOrthogonal * getOrthogonal();

  const std::map<std::string, std::shared_ptr<smacc2::ISmaccOrthogonal>> & getOrthogonals() const;

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage);

  template <typename EventType>
  void postEvent(EventType * ev, EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);

  template <typename EventType>
  void postEvent(EventLifeTime evlifetime = EventLifeTime::ABSOLUTE);

  template <typename T>
  bool getGlobalSMData(std::string name, T & ret);

  template <typename T>
  void setGlobalSMData(std::string name, T value);

  template <typename StateField, typename BehaviorType>
  void mapBehavior();

  std::string getStateMachineName();

  void state_machine_visualization();

  inline std::shared_ptr<SmaccStateInfo> getCurrentStateInfo() { return currentStateInfo_; }

  void publishTransition(const SmaccTransitionInfo & transitionInfo);

  /// this function should be implemented by the user to create the orthogonals
  virtual void onInitialize();

  void getTransitionLogHistory(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<smacc2_msgs::srv::SmaccGetTransitionHistory::Request> req,
    std::shared_ptr<smacc2_msgs::srv::SmaccGetTransitionHistory::Response> res);

  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection createSignalConnection(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object);

  // template <typename TSmaccSignal, typename TMemberFunctionPrototype>
  // boost::signals2::connection createSignalConnection(TSmaccSignal &signal, TMemberFunctionPrototype callback);

  template <typename StateType>
  void notifyOnStateEntryStart(StateType * state);

  template <typename StateType>
  void notifyOnStateEntryEnd(StateType * state);

  template <typename StateType>
  void notifyOnRuntimeConfigured(StateType * state);

  template <typename StateType>
  void notifyOnStateExitting(StateType * state);

  template <typename StateType>
  void notifyOnStateExited(StateType * state);

  template <typename StateType>
  void notifyOnRuntimeConfigurationFinished(StateType * state);

  inline unsigned long getCurrentStateCounter() const;

  inline ISmaccState * getCurrentState() const;

  inline const SmaccStateMachineInfo & getStateMachineInfo();

  template <typename InitialStateType>
  void buildStateMachineInfo();

  rclcpp::Node::SharedPtr getNode();

  inline rclcpp::Logger getLogger() { return nh_->get_logger(); }

protected:
  void checkStateMachineConsistence();

  void initializeROS(std::string smshortname);

  void onInitialized();

  template <typename TOrthogonal>
  void createOrthogonal();

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool getParam(std::string param_name, T & param_storage);

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  void setParam(std::string param_name, T param_val);

  // Delegates to ROS param access with the current NodeHandle
  template <typename T>
  bool param(std::string param_name, T & param_val, const T & default_val);

  // The node handle for this state
  rclcpp::Node::SharedPtr nh_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccStateMachine>::SharedPtr stateMachinePub_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccStatus>::SharedPtr stateMachineStatusPub_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccTransitionLogEntry>::SharedPtr transitionLogPub_;
  rclcpp::Service<smacc2_msgs::srv::SmaccGetTransitionHistory>::SharedPtr transitionHistoryService_;

  // if it is null, you may be located in a transition. There is a small gap of time where internally
  // this currentState_ is null. This may change in the future.
  ISmaccState * currentState_;

  std::shared_ptr<SmaccStateInfo> currentStateInfo_;

  smacc2_msgs::msg::SmaccStatus status_msg_;

  // orthogonals
  std::map<std::string, std::shared_ptr<smacc2::ISmaccOrthogonal>> orthogonals_;

private:
  std::recursive_mutex m_mutex_;
  std::recursive_mutex eventQueueMutex_;

  StateMachineInternalAction stateMachineCurrentAction;

  std::list<boost::signals2::connection> stateCallbackConnections;

  // shared variables
  std::map<std::string, std::pair<std::function<std::string()>, boost::any>> globalData_;

  std::vector<smacc2_msgs::msg::SmaccTransitionLogEntry> transitionLogHistory_;

  smacc2::SMRunMode runMode_;

  // Event to notify to the signaldetection thread that a request has been created...
  SignalDetector * signalDetector_;

  unsigned long stateSeqCounter_;

  void lockStateMachine(std::string msg);

  void unlockStateMachine(std::string msg);

  template <typename EventType>
  void propagateEventToStateReactors(ISmaccState * st, EventType * ev);

  std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo_;

  void updateStatusMessage();

  friend class ISmaccState;
  friend class SignalDetector;
};
}  // namespace smacc2

#include <smacc2/impl/smacc_client_impl.hpp>
#include <smacc2/impl/smacc_component_impl.hpp>
#include <smacc2/impl/smacc_orthogonal_impl.hpp>
#include <smacc2/impl/smacc_state_impl.hpp>
