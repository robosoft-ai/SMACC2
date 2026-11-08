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
#include <functional>
#include <memory>
#include <smacc2/introspection/smacc_type_info.hpp>
#include <smacc2/smacc_types.hpp>
#include <vector>
namespace smacc2
{
namespace introspection
{
// Ownership of the introspection graph. The SmaccStateMachineInfo owns every SmaccStateInfo
// through its `states` map; a SmaccStateInfo owns its `children_` and its `transitions_`; the
// static maps SmaccStateInfo::stateReactorsInfo and eventGeneratorsInfo own the reactor and
// generator infos, which own their handlers. Every pointer in the other direction (a state to its
// machine or parent, a transition to its states, a handler to its info) is a std::weak_ptr, so the
// graph is released when the state machine releases its SmaccStateMachineInfo. Readers of a weak
// pointer lock it; all of them run while the state machine is alive.
//-------------------------------------------------------------
struct ClientBehaviorInfoEntry
{
  std::function<void(smacc2::ISmaccState *)> factoryFunction;
  const std::type_info * behaviorType;
  const std::type_info * orthogonalType;
};

//---------------------------------------------
// contains information about an specific event in the statechart
struct SmaccEventInfo
{
  SmaccEventInfo(std::shared_ptr<TypeInfo> eventType);

  std::string getEventTypeName();

  std::string getEventSourceName();

  // AKA orthogonal
  std::string getOrthogonalName();

  std::string label;

  std::shared_ptr<TypeInfo> eventType;

private:
};

// contains metainformation about some specific transition in the statechart
struct SmaccTransitionInfo
{
  SmaccTransitionInfo() {}

  // Default-initialized: the startup transition-log entry in SmaccStateMachineBase::initiate_impl
  // builds this struct with make_shared and never assigns these two, and the user-provided
  // constructor leaves them indeterminate otherwise.
  bool historyNode = false;
  int index = 0;
  // Weak: the source state owns this transition, and the destiny state may own a transition back.
  std::weak_ptr<const SmaccStateInfo> sourceState;
  std::weak_ptr<const SmaccStateInfo> destinyState;

  std::string transitionTag;
  std::string transitionType;
  std::shared_ptr<SmaccEventInfo> eventInfo;

  smacc2::introspection::TypeInfo::Ptr transitionTypeInfo;
};
//---------------------------------------------

struct StateReactorCallbackFunctor
{
  std::function<void(std::shared_ptr<smacc2::StateReactor>)> fn;
};

// This object is used in a static context (when the runtime state reactor is not built yet)
class StateReactorHandler
{
private:
  std::vector<StateReactorCallbackFunctor> callbacks_;

public:
  // Only the node's logger is kept. This object is owned by a static map and outlives the state
  // machine; holding the node here would make a static the node's last owner and destroy it after
  // rclcpp::shutdown() has finalized the DDS participant.
  explicit StateReactorHandler(rclcpp::Node::SharedPtr nh)
  : logger_(nh ? nh->get_logger() : rclcpp::get_logger("StateReactorHandler"))
  {
  }

  void configureStateReactor(std::shared_ptr<smacc2::StateReactor> sr);

  template <typename TEv>
  void addInputEvent();

  template <typename TEv>
  void setOutputEvent();

  // Weak: the SmaccStateReactorInfo owns this handler (and its factoryFunction captures it).
  std::weak_ptr<smacc2::introspection::SmaccStateReactorInfo> srInfo_;

private:
  rclcpp::Logger logger_;
};
//-------------------------------------------------------------

// This object is used in a static context (when the runtime client behavior is not built yet)
template <class T>
class ClientBehaviorHandler
{
public:
  ClientBehaviorHandler() {}
};
//-------------------------------------------------------------
// contains metainformation about some specific state reactor in the statechart
struct SmaccStateReactorInfo
{
  // Never assigned or read by the core; kept weak so that it can never own a state.
  std::weak_ptr<SmaccStateInfo> ownerState;
  std::function<void(smacc2::ISmaccState *)> factoryFunction;

  std::shared_ptr<TypeInfo> stateReactorType;
  std::shared_ptr<TypeInfo> outputEventType;
  std::shared_ptr<TypeInfo> objectTagType;
  std::vector<std::shared_ptr<SmaccEventInfo>> sourceEventTypes;
  std::shared_ptr<StateReactorHandler> srh;
};

//-------------------------------------------------------------
struct EventGeneratorCallbackFunctor
{
  std::function<void(std::shared_ptr<smacc2::SmaccEventGenerator>)> fn;
};

class EventGeneratorHandler
{
private:
  std::vector<EventGeneratorCallbackFunctor> callbacks_;

public:
  void configureEventGenerator(std::shared_ptr<smacc2::SmaccEventGenerator> eg);

  template <typename TEv>
  void setOutputEvent();

  // Weak: the SmaccEventGeneratorInfo owns this handler (and its factoryFunction captures it).
  std::weak_ptr<smacc2::introspection::SmaccEventGeneratorInfo> egInfo_;
};

// contains metainformation about some specific event generator in the statechart
struct SmaccEventGeneratorInfo
{
  // Never assigned or read by the core; kept weak so that it can never own a state.
  std::weak_ptr<SmaccStateInfo> ownerState;
  std::function<void(smacc2::ISmaccState *)> factoryFunction;

  std::shared_ptr<TypeInfo> eventGeneratorType;
  std::shared_ptr<TypeInfo> outputEventType;
  std::shared_ptr<TypeInfo> objectTagType;
  std::vector<std::shared_ptr<SmaccEventInfo>> sourceEventTypes;
  std::shared_ptr<EventGeneratorHandler> egh;
};
//-----------------------------------------------------------------------
enum class SmaccStateType
{
  SUPERSTATE = 2,
  STATE = 1,
  SUPERSTATE_ROUTINE = 1
};
//-----------------------------------------------------------------------
class SmaccStateInfo : public std::enable_shared_from_this<SmaccStateInfo>
{
public:
  typedef std::shared_ptr<SmaccStateInfo> Ptr;

  static std::map<const std::type_info *, std::vector<ClientBehaviorInfoEntry>> staticBehaviorInfo;
  static std::map<const std::type_info *, std::vector<std::shared_ptr<SmaccStateReactorInfo>>>
    stateReactorsInfo;
  static std::map<const std::type_info *, std::vector<std::shared_ptr<SmaccEventGeneratorInfo>>>
    eventGeneratorsInfo;

  int stateIndex_;
  std::string fullStateName;
  std::string demangledStateName;

  // Weak back-references: the machine info owns this state through `states`, and the parent owns
  // it through `children_`. Use getStateMachineInfo() and getParentState().
  std::weak_ptr<SmaccStateMachineInfo> stateMachine_;
  std::weak_ptr<SmaccStateInfo> parentState_;
  std::vector<SmaccTransitionInfo> transitions_;

  std::vector<std::shared_ptr<SmaccStateInfo>> children_;
  int depth_ = 0;  // root state (no parent) is depth 0; the constructor adds 1 per ancestor
  const std::type_info * tid_;

  SmaccStateInfo(
    const std::type_info * tid, std::shared_ptr<SmaccStateInfo> parentState,
    std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo);

  SmaccStateType getStateLevel();

  // Null once the SmaccStateMachineInfo has been destroyed, which only happens after the state
  // machine itself is gone.
  inline std::shared_ptr<SmaccStateMachineInfo> getStateMachineInfo() const
  {
    return stateMachine_.lock();
  }

  // Null for the root state, and once the parent has been destroyed.
  inline std::shared_ptr<SmaccStateInfo> getParentState() const { return parentState_.lock(); }

  inline int depth() const { return depth_; }

  void getAncestors(std::list<const SmaccStateInfo *> & ancestorsList) const;

  std::string getFullPath();

  template <typename StateType>
  std::shared_ptr<SmaccStateInfo> createChildState();

  template <typename EvType>
  void declareTransition(
    std::shared_ptr<SmaccStateInfo> & dstState, std::string transitionTag,
    std::string transitionType, bool history, TypeInfo::Ptr transitionTypeInfo);

  const std::string & toShortName() const;

  std::string getDemangledFullName() const;
  rclcpp::Node::SharedPtr getNode();
  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }
};
}  // namespace introspection
}  // namespace smacc2
