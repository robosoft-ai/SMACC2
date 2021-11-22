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
#include <functional>
#include <memory>
#include <smacc2/introspection/smacc_type_info.hpp>
#include <smacc2/smacc_types.hpp>
#include <vector>
namespace smacc2
{
namespace introspection
{
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

  bool historyNode;
  int index;
  std::shared_ptr<const SmaccStateInfo> sourceState;
  std::shared_ptr<const SmaccStateInfo> destinyState;

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

class StateReactorHandler
{
private:
  std::vector<StateReactorCallbackFunctor> callbacks_;

public:
  StateReactorHandler(rclcpp::Node::SharedPtr nh) : nh_(nh) {}

  void configureStateReactor(std::shared_ptr<smacc2::StateReactor> sr);

  template <typename TEv>
  void addInputEvent();

  template <typename TEv>
  void setOutputEvent();

  rclcpp::Node::SharedPtr getNode();

  std::shared_ptr<smacc2::introspection::SmaccStateReactorInfo> srInfo_;

private:
  rclcpp::Node::SharedPtr nh_;
};

//-------------------------------------------------------------
// contains metainformation about some specific state reactor in the statechart
struct SmaccStateReactorInfo
{
  std::shared_ptr<SmaccStateInfo> ownerState;
  std::function<void(smacc2::ISmaccState *)> factoryFunction;

  //const std::type_info *stateReactorType;
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

  std::shared_ptr<smacc2::introspection::SmaccEventGeneratorInfo> egInfo_;
};

// contains metainformation about some specific event generator in the statechart
struct SmaccEventGeneratorInfo
{
  std::shared_ptr<SmaccStateInfo> ownerState;
  std::function<void(smacc2::ISmaccState *)> factoryFunction;

  // const std::type_info *eventGeneratorType;
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

  std::shared_ptr<SmaccStateMachineInfo> stateMachine_;
  std::shared_ptr<SmaccStateInfo> parentState_;
  std::vector<SmaccTransitionInfo> transitions_;

  std::vector<std::shared_ptr<SmaccStateInfo>> children_;
  int depth_;
  const std::type_info * tid_;

  SmaccStateInfo(
    const std::type_info * tid, std::shared_ptr<SmaccStateInfo> parentState,
    std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo);

  SmaccStateType getStateLevel();

  inline int depth() const { return depth_; }

  void getAncestors(std::list<const SmaccStateInfo *> & ancestorsList) const;

  std::string getFullPath();

  template <typename StateType>
  std::shared_ptr<SmaccStateInfo> createChildState();

  template <typename EvType>
  void declareTransition(
    std::shared_ptr<SmaccStateInfo> & dstState, std::string transitionTag,
    std::string transitionType, bool history, TypeInfo::Ptr transitionTypeInfo);

  // template <typename EvSource, template <typename> typename EvType>
  // void declareTransition(std::shared_ptr<SmaccStateInfo> &dstState, std::string transitionTag, std::string transitionType, bool history);

  const std::string & toShortName() const;

  std::string getDemangledFullName() const;
  rclcpp::Node::SharedPtr getNode();
  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }
};
}  // namespace introspection
}  // namespace smacc2
