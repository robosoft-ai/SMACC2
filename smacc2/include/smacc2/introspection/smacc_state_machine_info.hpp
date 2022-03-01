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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <smacc2/common.hpp>
#include <smacc2/smacc_orthogonal.hpp>

// smacc_msgs
#include <smacc2_msgs/msg/smacc_event_generator.hpp>
#include <smacc2_msgs/msg/smacc_orthogonal.hpp>
#include <smacc2_msgs/msg/smacc_state.hpp>
#include <smacc2_msgs/msg/smacc_state_reactor.hpp>
#include <smacc2_msgs/msg/smacc_transition.hpp>

namespace smacc2
{
namespace introspection
{
extern rclcpp::Node::SharedPtr globalNh_;

class SmaccStateMachineInfo : public std::enable_shared_from_this<SmaccStateMachineInfo>
{
public:
  explicit SmaccStateMachineInfo(rclcpp::Node::SharedPtr nh) : nh_(nh) {}

  std::map<std::string, std::shared_ptr<SmaccStateInfo>> states;

  std::vector<smacc2_msgs::msg::SmaccState> stateMsgs;

  template <typename InitialStateType>
  void buildStateMachineInfo();

  template <typename StateType>
  std::shared_ptr<SmaccStateInfo> createState(std::shared_ptr<SmaccStateInfo> parentState);

  template <typename StateType>
  bool containsState()
  {
    auto typeNameStr = typeid(StateType).name();

    return states.count(typeNameStr) > 0;
  }

  template <typename StateType>
  std::shared_ptr<SmaccStateInfo> getState()
  {
    if (this->containsState<StateType>())
    {
      return states[typeid(StateType).name()];
    }
    return nullptr;
  }

  inline rclcpp::Node::SharedPtr getNode() { return nh_; }

  inline rclcpp::Logger getLogger() { return nh_->get_logger(); }

  template <typename StateType>
  void addState(std::shared_ptr<StateType> & state);
  void assembleSMStructureMessage(ISmaccStateMachine * sm);

private:
  rclcpp::Node::SharedPtr nh_;
};

//---------------------------------------------
struct AddSubState
{
  std::shared_ptr<SmaccStateInfo> & parentState_;
  explicit AddSubState(std::shared_ptr<SmaccStateInfo> & parentState) : parentState_(parentState) {}

  template <typename T>
  void operator()(T);
};

//---------------------------------------------
struct AddTransition
{
  std::shared_ptr<SmaccStateInfo> & currentState_;

  explicit AddTransition(std::shared_ptr<SmaccStateInfo> & currentState)
  : currentState_(currentState)
  {
  }

  template <
    template <typename, typename, typename> typename TTransition, typename TevSource,
    template <typename> typename EvType, typename Tag, typename DestinyState>
  void operator()(TTransition<EvType<TevSource>, DestinyState, Tag>);

  template <
    template <typename, typename> typename TTransition, typename TevSource,
    template <typename> typename EvType, typename DestinyState>
  void operator()(TTransition<EvType<TevSource>, DestinyState>);

  template <typename T>
  void operator()(T);
};

//---------------------------------------------
template <typename InitialStateType>
struct WalkStatesExecutor
{
  static void walkStates(std::shared_ptr<SmaccStateInfo> & currentState, bool rootInitialNode);
};

//---------------------------------------------
template <typename T>
void AddSubState::operator()(T)
{
  using type_t = typename T::type;
  //auto childState = this->parentState_->createChildState<type_t>()
  WalkStatesExecutor<type_t>::walkStates(parentState_, false);
}

//--------------------------------------------
template <typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type processSubState(
  std::shared_ptr<SmaccStateInfo> & parentState)
{
  WalkStatesExecutor<T>::walkStates(parentState, false);
}

//---------------------------------------------
template <typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type processSubState(
  std::shared_ptr<SmaccStateInfo> & parentState)
{
  using boost::mpl::_1;
  using wrappedList = typename boost::mpl::transform<T, add_type_wrapper<_1>>::type;
  boost::mpl::for_each<wrappedList>(AddSubState(parentState));
}

//--------------------------------------------
/*Iterate on All Transitions*/
template <typename T>
typename enable_if<boost::mpl::is_sequence<T>>::type processTransitions(
  std::shared_ptr<SmaccStateInfo> & sourceState)
{
  RCLCPP_INFO(
    globalNh_->get_logger(), "State %s Walker has transition list",
    sourceState->fullStateName.c_str());
  using boost::mpl::_1;
  using wrappedList = typename boost::mpl::transform<T, add_type_wrapper<_1>>::type;
  boost::mpl::for_each<wrappedList>(AddTransition(sourceState));
}

template <typename Ev, typename Dst, typename Tag>
void processTransition(
  smacc2::Transition<Ev, boost::statechart::deep_history<Dst>, Tag> *,
  std::shared_ptr<SmaccStateInfo> & sourceState)
{
  auto transitionTypeInfo = TypeInfo::getTypeInfoFromType<
    smacc2::Transition<Ev, boost::statechart::deep_history<Dst>, Tag>>();
  smacc2::Transition<Ev, Dst, Tag> * mock = nullptr;
  processTransitionAux(mock, sourceState, true, transitionTypeInfo);
}

template <typename Ev, typename Dst, typename Tag>
void processTransition(
  smacc2::Transition<Ev, Dst, Tag> * t, std::shared_ptr<SmaccStateInfo> & sourceState)
{
  auto transitionTypeInfo = TypeInfo::getTypeInfoFromType<smacc2::Transition<Ev, Dst, Tag>>();
  RCLCPP_INFO(
    globalNh_->get_logger(), "State %s Walker transition: %s", sourceState->toShortName().c_str(),
    demangleSymbol(typeid(Ev).name()).c_str());
  processTransitionAux(t, sourceState, false, transitionTypeInfo);
}

template <typename Ev, typename Dst, typename Tag>
void processTransitionAux(
  smacc2::Transition<Ev, Dst, Tag> *, std::shared_ptr<SmaccStateInfo> & sourceState, bool history,
  TypeInfo::Ptr & transitionTypeInfo)
{
  RCLCPP_INFO(
    globalNh_->get_logger(), "State %s Walker transition: %s", sourceState->toShortName().c_str(),
    demangleSymbol(typeid(Ev).name()).c_str());
  std::string transitionTag;
  std::string transitionType;

  if (typeid(Tag) != typeid(default_transition_name))
  {
    transitionTag = demangleSymbol<Tag>();
    transitionType = getTransitionType<Tag>();
    RCLCPP_DEBUG_STREAM(globalNh_->get_logger(), "TRANSITION TYPE:" << transitionType);
  }
  else
  {
    transitionTag = "";
    automaticTransitionTag<Ev>(transitionTag);
    automaticTransitionType<Ev>(transitionType);
  }

  RCLCPP_INFO_STREAM(globalNh_->get_logger(), "Transition tag: " << transitionTag);

  if (!sourceState->stateMachine_->containsState<Dst>())
  {
    auto realparentState = sourceState->stateMachine_->getState<typename Dst::TContext>();
    auto siblingnode = sourceState->stateMachine_->createState<Dst>(realparentState);

    // auto siblingnode = sourceState->stateMachine_->createState<Dst>(sourceState->parentState_);
    WalkStatesExecutor<Dst>::walkStates(siblingnode, true);
    sourceState->declareTransition<Ev>(
      siblingnode, transitionTag, transitionType, history, transitionTypeInfo);
  }
  else
  {
    // auto realparentState = sourceState->stateMachine_->getState<typename Dst::TContext>();
    // auto siblingnode = sourceState->stateMachine_->createState<Dst>(realparentState);

    auto siblingnode = sourceState->stateMachine_->getState<Dst>();
    sourceState->declareTransition<Ev>(
      siblingnode, transitionTag, transitionType, history, transitionTypeInfo);
  }
}

//---------------------------------------------
template <typename EvType>
void SmaccStateInfo::declareTransition(
  std::shared_ptr<SmaccStateInfo> & dstState, std::string transitionTag, std::string transitionType,
  bool history, TypeInfo::Ptr transitionTypeInfo)
{
  auto evtype = demangledTypeName<EvType>();

  SmaccTransitionInfo transitionInfo;
  transitionInfo.index = transitions_.size();
  transitionInfo.sourceState = shared_from_this();
  transitionInfo.destinyState = dstState;
  transitionInfo.transitionTypeInfo = transitionTypeInfo;

  if (transitionTag != "")
    transitionInfo.transitionTag = transitionTag;
  else
    transitionInfo.transitionTag = "Transition_" + std::to_string(transitionInfo.index);

  transitionInfo.transitionType = transitionType;
  transitionInfo.historyNode = history;

  transitionInfo.eventInfo =
    std::make_shared<SmaccEventInfo>(transitionTypeInfo->templateParameters.front());

  EventLabel<EvType>(transitionInfo.eventInfo->label);
  // RCLCPP_DEBUG_STREAM(getLogger(), "LABEL: " << transitionInfo.eventInfo->label);

  transitions_.push_back(transitionInfo);
}
//---------------------------------------------

//------------------ staticConfigure -----------------------------

// SFINAE test
template <typename T>
class HasOnDefinition
{
private:
  typedef char YesType[1];
  typedef char NoType[2];

  template <typename C>
  static YesType & test(decltype(&C::staticConfigure));
  template <typename C>
  static NoType & test(...);

public:
  enum
  {
    value = sizeof(test<T>(0)) == sizeof(YesType)
  };
};

// template <typename TevSource, template <typename> typename EvType>
// void SmaccStateInfo::declareTransition(std::shared_ptr<SmaccStateInfo> &dstState, std::string transitionTag, std::string transitionType, bool history)
// {
//     auto evtype = demangledTypeName<EvType<TevSource>>();

//     SmaccTransitionInfo transitionInfo;
//     transitionInfo.index = transitions_.size();
//     transitionInfo.sourceState = shared_from_this();
//     transitionInfo.destinyState = dstState;

//     if (transitionTag != "")
//         transitionInfo.transitionTag = transitionTag;
//     else
//         transitionInfo.transitionTag = "Transition_" + std::to_string(transitionInfo.index);

//     transitionInfo.transitionType = transitionType;

//     transitionInfo.eventInfo = std::make_shared<SmaccEventInfo>(TypeInfo::getTypeInfoFromString(demangleSymbol(typeid(EvType<TevSource>).name())));

//     EventLabel<EvType<TevSource>>(transitionInfo.eventInfo->label);
//     RCLCPP_ERROR_STREAM(getLogger(),"LABEL: " << transitionInfo.eventInfo->label);

//     transitions_.push_back(transitionInfo);
// }

//---------------------------------------------
template <typename Ev, typename Dst>
void processTransition(
  statechart::transition<Ev, Dst> *, std::shared_ptr<SmaccStateInfo> & sourceState)
{
  // RCLCPP_INFO_STREAM(getLogger(),"GOTCHA");
}

template <typename Ev>
void processTransition(
  statechart::custom_reaction<Ev> *, std::shared_ptr<SmaccStateInfo> & sourceState)
{
  // RCLCPP_INFO_STREAM(getLogger(),"GOTCHA");
}

//---------------------------------------------
// only reached if it is a leaf transition in the mpl::list

template <typename T>
typename disable_if<boost::mpl::is_sequence<T>>::type processTransitions(
  std::shared_ptr<SmaccStateInfo> & sourceState)
{
  // RCLCPP_INFO_STREAM(getLogger(),"state transition from: " << sourceState->demangledStateName <<
  // " of type: " << demangledTypeName<T>());
  T * dummy = nullptr;
  processTransition(dummy, sourceState);
}

template <typename T>
typename std::enable_if<HasOnDefinition<T>::value, void>::type CallOnDefinition()
{
  /* something when T has toString ... */
  RCLCPP_INFO_STREAM(
    globalNh_->get_logger(), "EXECUTING ONDEFINITION: " << demangleSymbol(typeid(T).name()));
  T::staticConfigure();
}

template <typename T>
typename std::enable_if<!HasOnDefinition<T>::value, void>::type CallOnDefinition()
{
  RCLCPP_INFO_STREAM(
    globalNh_->get_logger(),
    "static OnDefinition: dont exist for " << demangleSymbol(typeid(T).name()));
  /* something when T has toString ... */
}

/*
// only reached if it is a leaf transition in the mpl::list
template <template <typename,typename,typename> typename TTransition, typename TevSource,
template <typename> typename EvType, typename Tag, typename DestinyState >
typename disable_if<boost::mpl::is_sequence<TTransition<EvType<TevSource>,DestinyState, Tag>>>::type
processTransitions(std::shared_ptr<SmaccStateInfo> &sourceState)
{
    RCLCPP_INFO(getLogger(),"DETECTED COMPLEX TRANSITION **************");
    // RCLCPP_INFO_STREAM(getLogger(),"state transition from: " << sourceState->demangledStateName
    << " of type: " << demangledTypeName<T>());
    TTransition<EvType<TevSource>,DestinyState, Tag> *dummy;
    processTransition(dummy, sourceState);
}

template <template <typename,typename> typename TTransition, typename TevSource,
template <typename> typename EvType, typename DestinyState >
typename disable_if<boost::mpl::is_sequence<TTransition<EvType<TevSource>,DestinyState>>>::type
processTransitions(std::shared_ptr<SmaccStateInfo> &sourceState)
{
    RCLCPP_INFO(getLogger(),"DETECTED COMPLEX TRANSITION **************");
    // RCLCPP_INFO_STREAM(getLogger(),"state transition from: " << sourceState->demangledStateName
    << " of type: " << demangledTypeName<T>());
    TTransition<EvType<TevSource>,DestinyState> *dummy;
    processTransition(dummy, sourceState);
}
*/

//--------------------------------------------

template <
  template <typename, typename, typename> typename TTransition, typename TevSource,
  template <typename> typename EvType, typename Tag, typename DestinyState>
void AddTransition::operator()(TTransition<EvType<TevSource>, DestinyState, Tag>)
{
  processTransitions<TTransition<EvType<TevSource>, DestinyState, Tag>>(currentState_);
}

//--------------------------------------------

template <
  template <typename, typename> typename TTransition, typename TevSource,
  template <typename> typename EvType, typename DestinyState>
void AddTransition::operator()(TTransition<EvType<TevSource>, DestinyState>)
{
  processTransitions<TTransition<EvType<TevSource>, DestinyState>>(currentState_);
}

template <typename TTrans>
void AddTransition::operator()(TTrans)
{
  using type_t = typename TTrans::type;
  processTransitions<type_t>(currentState_);
}

/*
void CallOnDefinition(...)
{

}*/

//-----------------------------------------------------------------------------------
template <typename InitialStateType>
void WalkStatesExecutor<InitialStateType>::walkStates(
  std::shared_ptr<SmaccStateInfo> & parentState, bool rootInitialNode)
{
  //rclcpp::Duration(1).sleep();
  auto currentname = demangledTypeName<InitialStateType>();

  std::shared_ptr<SmaccStateInfo> targetState;

  if (!rootInitialNode)
  {
    if (parentState->stateMachine_->containsState<InitialStateType>())
    {
      // it already exist: break;
      return;
    }

    targetState = parentState->createChildState<InitialStateType>();
  }
  else
  {
    targetState = parentState;
  }

  CallOnDefinition<InitialStateType>();

  typedef
    typename std::remove_pointer<decltype(InitialStateType::smacc_inner_type)>::type InnerType;
  processSubState<InnerType>(targetState);

  // -------------------- REACTIONS --------------------
  typedef typename InitialStateType::reactions reactions;
  // RCLCPP_INFO_STREAM(getLogger(),"state machine initial state reactions: "
  // << demangledTypeName<reactions>());

  processTransitions<reactions>(targetState);
}

//------------------------------------------------

template <typename InitialStateType>
void SmaccStateMachineInfo::buildStateMachineInfo()
{
  auto initialState = this->createState<InitialStateType>(nullptr);
  WalkStatesExecutor<InitialStateType>::walkStates(initialState, true);
}

template <typename StateType>
std::shared_ptr<SmaccStateInfo> SmaccStateMachineInfo::createState(
  std::shared_ptr<SmaccStateInfo> parent)
{
  auto thisptr = this->shared_from_this();
  auto * statetid = &(typeid(StateType));

  auto demangledName = demangledTypeName<StateType>();
  RCLCPP_INFO_STREAM(getLogger(), "Creating State Info: " << demangledName);

  auto state = std::make_shared<SmaccStateInfo>(statetid, parent, thisptr);
  state->demangledStateName = demangledName;
  state->fullStateName = typeid(StateType).name();
  state->stateIndex_ = states.size();

  if (parent != nullptr)
  {
    parent->children_.push_back(state);
  }

  this->addState(state);

  return state;
}

template <typename StateType>
void SmaccStateMachineInfo::addState(std::shared_ptr<StateType> & state)
{
  states[state->fullStateName] = state;
}

template <typename StateType>
std::shared_ptr<SmaccStateInfo> SmaccStateInfo::createChildState()
{
  auto realparentState = this->stateMachine_->getState<typename StateType::TContext>();

  auto childState = this->stateMachine_->createState<StateType>(realparentState);

  RCLCPP_WARN_STREAM(
    getLogger(), "Real parent state> " << demangleSymbol<typename StateType::TContext>());

  /*auto contextInfo = TypeInfo::getTypeInfoFromType<InitialStateType>();
    auto parentState2= getState<InitialStateType::TContext>();
    parentState2->createChildState<InitialStateType>();*/

  // this->stateMachine_->addState(childState);
  // stateMachineInfo.addState(stateMachineInfo)
  // stateNames.push_back(currentname);
  // RCLCPP_INFO(getLogger(),"------------");
  // RCLCPP_INFO_STREAM(getLogger(),"** STATE state: "<< this->demangledStateName);

  return childState;
}
}  // namespace introspection
}  // namespace smacc2
