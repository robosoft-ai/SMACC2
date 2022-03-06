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

namespace smacc2
{
class ISmaccState
{
public:
  virtual ISmaccStateMachine & getStateMachine() = 0;

  virtual std::string getName() = 0;

  inline ISmaccState * getParentState() { return parentState_; }

  inline rclcpp::Node::SharedPtr & getNode() { return node_; }

  inline rclcpp::Logger getLogger() { return *logger_; }

  virtual std::string getClassName();

  template <typename TOrthogonal, typename TBehavior, typename... Args>
  std::shared_ptr<TBehavior> configure(Args &&... args);

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage);

  template <typename SmaccClientType>
  void requiresClient(SmaccClientType *& storage);

  template <typename T>
  bool getGlobalSMData(std::string name, T & ret);

  // Store globally in this state machine. (By value parameter )
  template <typename T>
  void setGlobalSMData(std::string name, T value);

  template <
    typename TStateReactor, typename TTriggerEvent, typename TEventList, typename... TEvArgs>
  std::shared_ptr<TStateReactor> createStateReactor(TEvArgs... args);

  template <typename TStateReactor, typename... TEvArgs>
  std::shared_ptr<TStateReactor> createStateReactor(TEvArgs... args);

  template <typename TEventGenerator, typename... TEvArgs>
  std::shared_ptr<TEventGenerator> createEventGenerator(TEvArgs... args);

  template <typename EventType>
  void postEvent(const EventType & ev);

  template <typename EventType>
  void postEvent();

  // used for transition logging
  template <typename TransitionType>
  void notifyTransition();

  // used for transition logging
  void notifyTransitionFromTransitionTypeInfo(
    std::shared_ptr<smacc2::introspection::TypeInfo> & transitionTypeInfo);

  inline std::vector<std::shared_ptr<StateReactor>> & getStateReactors() { return stateReactors_; }

  inline std::vector<std::shared_ptr<SmaccEventGenerator>> & getEventGenerators()
  {
    return eventGenerators_;
  }

  template <typename TOrthogonal>
  TOrthogonal * getOrthogonal();

  template <typename TEventGenerator>
  TEventGenerator * getEventGenerator();

  template <typename TStateReactor>
  TStateReactor * getStateReactor();

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::vector<std::shared_ptr<StateReactor>> stateReactors_;
  std::vector<std::shared_ptr<smacc2::SmaccEventGenerator>> eventGenerators_;

  ISmaccState * parentState_;

  const smacc2::introspection::SmaccStateInfo * stateInfo_;
};
}  // namespace smacc2
