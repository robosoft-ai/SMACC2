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

/**************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 **************************************************************************************************/

#pragma once

#include <memory>
#include <sstream>
#include <string>

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc_event_generator.hpp>
#include <smacc2/smacc_orthogonal.hpp>
#include <smacc2/smacc_signal_detector.hpp>
#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_machine.hpp>
#include <smacc2/smacc_state_reactor.hpp>

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc_orthogonal.hpp>
#include <smacc2/smacc_signal_detector.hpp>
#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_machine.hpp>
#include <smacc2/smacc_state_reactor.hpp>

#include <boost/function_types/function_arity.hpp>
#include <boost/function_types/function_type.hpp>
#include <boost/function_types/parameter_types.hpp>
#include <smacc2/smacc_tracing/smacc_tracing.hpp>
#include <smacc2_msgs/msg/smacc_status.hpp>

namespace smacc2
{
using namespace smacc2::introspection;

template <typename TOrthogonal>
TOrthogonal * ISmaccStateMachine::getOrthogonal()
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex_);

  std::string orthogonalkey = demangledTypeName<TOrthogonal>();
  TOrthogonal * ret;

  auto it = orthogonals_.find(orthogonalkey);

  if (it != orthogonals_.end())
  {
    RCLCPP_DEBUG(
      getLogger(),
      "Orthogonal %s resource is being required from some state, client or component. Found "
      "resource in "
      "cache.",
      orthogonalkey.c_str());
    ret = dynamic_cast<TOrthogonal *>(it->second.get());
    return ret;
  }
  else
  {
    std::stringstream ss;
    ss << "Orthogonal not found " << orthogonalkey.c_str() << std::endl;
    ss << "The existing orthogonals are the following: " << std::endl;
    for (auto & orthogonal : orthogonals_)
    {
      ss << " - " << orthogonal.first << std::endl;
    }

    RCLCPP_WARN_STREAM(getLogger(), ss.str());

    return nullptr;
  }
}

//-------------------------------------------------------------------------------------------------------
template <typename TOrthogonal>
void ISmaccStateMachine::createOrthogonal()
{
  this->lockStateMachine("create orthogonal");
  std::string orthogonalkey = demangledTypeName<TOrthogonal>();

  if (orthogonals_.count(orthogonalkey) == 0)
  {
    auto ret = std::make_shared<TOrthogonal>();
    orthogonals_[orthogonalkey] = dynamic_pointer_cast<smacc2::ISmaccOrthogonal>(ret);

    ret->setStateMachine(this);

    RCLCPP_INFO(getLogger(), "%s Orthogonal is created", orthogonalkey.c_str());
  }
  else
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "There were already one existing orthogonal of type "
                     << orthogonalkey.c_str() << ". Skipping creation orthogonal request. ");
    std::stringstream ss;
    ss << "The existing orthogonals are the following: " << std::endl;
    for (auto & orthogonal : orthogonals_)
    {
      ss << " - " << orthogonal.first << std::endl;
    }
    RCLCPP_WARN_STREAM(getLogger(), ss.str());
  }
  this->unlockStateMachine("create orthogonal");
}

//-------------------------------------------------------------------------------------------------------
template <typename SmaccComponentType>
void ISmaccStateMachine::requiresComponent(SmaccComponentType *& storage)
{
  RCLCPP_DEBUG(
    getLogger(), "component %s is required",
    demangleSymbol(typeid(SmaccComponentType).name()).c_str());
  std::lock_guard<std::recursive_mutex> lock(m_mutex_);

  for (auto ortho : this->orthogonals_)
  {
    for (auto & client : ortho.second->clients_)
    {
      storage = client->getComponent<SmaccComponentType>();
      if (storage != nullptr)
      {
        return;
      }
    }
  }

  RCLCPP_WARN(
    getLogger(), "component %s is required but it was not found in any orthogonal",
    demangleSymbol(typeid(SmaccComponentType).name()).c_str());

  // std::string componentkey = demangledTypeName<SmaccComponentType>();
  // SmaccComponentType *ret;

  // auto it = components_.find(componentkey);

  // if (it == components_.end())
  // {
  //     RCLCPP_DEBUG(getLogger(),"%s smacc component is required. Creating a new instance.",
  //     componentkey.c_str());

  //     ret = new SmaccComponentType();
  //     ret->setStateMachine(this);
  //     components_[componentkey] = static_cast<smacc2::ISmaccComponent *>(ret);
  //     RCLCPP_DEBUG(getLogger(),"%s resource is required. Done.", componentkey.c_str());
  // }
  // else
  // {
  //     RCLCPP_DEBUG(getLogger(),"%s resource is required. Found resource in cache.",
  //     componentkey.c_str()); ret = dynamic_cast<SmaccComponentType *>(it->second);
  // }

  // storage = ret;
}
//-------------------------------------------------------------------------------------------------------
template <typename EventType>
void ISmaccStateMachine::postEvent(EventType * ev, EventLifeTime evlifetime)
{
  std::lock_guard<std::recursive_mutex> guard(eventQueueMutex_);

#define eventtypename demangleSymbol<EventType>().c_str()

  TRACEPOINT(smacc_event, eventtypename);

  if (
    evlifetime == EventLifeTime::CURRENT_STATE &&
    (stateMachineCurrentAction == StateMachineInternalAction::STATE_EXITING ||
     stateMachineCurrentAction == StateMachineInternalAction::TRANSITIONING))
  {
    RCLCPP_WARN_STREAM(
      getLogger(),
      "CURRENT STATE SCOPED EVENT SKIPPED, state is exiting/transitioning " << eventtypename);
    return;
    // in this case we may lose/skip events, if this is not right for some cases we should create a
    // queue to lock the events during the transitions. This issues appeared when a client
    // asyncbehavior was posting an event meanwhile we were doing the transition, but the main
    // thread was waiting for its correct finalization (with thread.join)
  }

  // when a postting event is requested by any component, client, or client behavior
  // we reach this place. Now, we propagate the events to all the state state reactors to generate
  // some more events

  RCLCPP_INFO_STREAM(getLogger(), "[PostEvent entry point] " << eventtypename);
  auto currentstate = currentState_;
  if (currentstate != nullptr)
  {
    propagateEventToStateReactors(currentstate, ev);
  }

  this->signalDetector_->postEvent(ev);
}

template <typename EventType>
void ISmaccStateMachine::postEvent(EventLifeTime evlifetime)
{
  auto * ev = new EventType();
  this->postEvent(ev, evlifetime);
}

template <typename T>
bool ISmaccStateMachine::getGlobalSMData(std::string name, T & ret)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex_);
  // RCLCPP_WARN(getLogger(),"get SM Data lock acquire");
  bool success = false;

  if (!globalData_.count(name))
  {
    // RCLCPP_WARN(getLogger(),"get SM Data - data do not exist");
    success = false;
  }
  else
  {
    // RCLCPP_WARN(getLogger(),"get SM DAta -data exist. accessing");
    try
    {
      auto & v = globalData_[name];

      // RCLCPP_WARN(getLogger(),"get SM DAta -data exist. any cast");
      ret = boost::any_cast<T>(v.second);
      success = true;
      // RCLCPP_WARN(getLogger(),"get SM DAta -data exist. success");
    }
    catch (boost::bad_any_cast & ex)
    {
      RCLCPP_ERROR(getLogger(), "bad any cast: %s", ex.what());
      success = false;
    }
  }

  // RCLCPP_WARN(getLogger(),"get SM Data lock release");
  return success;
}

template <typename T>
void ISmaccStateMachine::setGlobalSMData(std::string name, T value)
{
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex_);
    // RCLCPP_WARN(getLogger(),"set SM Data lock acquire");

    globalData_[name] = {
      [this, name]() {
        std::stringstream ss;
        auto val = any_cast<T>(globalData_[name].second);
        ss << val;
        return ss.str();
      },
      value};
  }

  this->updateStatusMessage();
}

template <typename StateField, typename BehaviorType>
void ISmaccStateMachine::mapBehavior()
{
  std::string stateFieldName = demangleSymbol(typeid(StateField).name());
  std::string behaviorType = demangleSymbol(typeid(BehaviorType).name());
  RCLCPP_INFO(
    getLogger(), "Mapping state field '%s' to stateReactor '%s'", stateFieldName.c_str(),
    behaviorType.c_str());
  SmaccClientBehavior * globalreference;
  if (!this->getGlobalSMData(stateFieldName, globalreference))
  {
    // Using the requires component approach, we force a unique existence
    // of this component
    BehaviorType * behavior;
    this->requiresComponent(behavior);
    globalreference = dynamic_cast<ISmaccClientBehavior *>(behavior);

    this->setGlobalSMData(stateFieldName, globalreference);
  }
}

namespace utils
{
template <int arity>
struct Bind
{
  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection bindaux(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object);
};

template <>
struct Bind<1>
{
  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection bindaux(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object)
  {
    return signal.connect([=]() { return (object->*callback)(); });
  }
};

template <>
struct Bind<2>
{
  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection bindaux(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object)
  {
    return signal.connect([=](auto a1) { return (object->*callback)(a1); });
  }
};

template <>
struct Bind<3>
{
  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection bindaux(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object)
  {
    return signal.connect([=](auto a1, auto a2) { return (object->*callback)(a1, a2); });
  }
};

template <>
struct Bind<4>
{
  template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
  boost::signals2::connection bindaux(
    TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object)
  {
    return signal.connect(
      [=](auto a1, auto a2, auto a3) { return (object->*callback)(a1, a2, a3); });
  }
};
}  // namespace utils
using namespace smacc2::utils;

template <typename TSmaccSignal, typename TMemberFunctionPrototype, typename TSmaccObjectType>
boost::signals2::connection ISmaccStateMachine::createSignalConnection(
  TSmaccSignal & signal, TMemberFunctionPrototype callback, TSmaccObjectType * object)
{
  static_assert(
    std::is_base_of<ISmaccState, TSmaccObjectType>::value ||
      std::is_base_of<ISmaccClient, TSmaccObjectType>::value ||
      std::is_base_of<ISmaccClientBehavior, TSmaccObjectType>::value ||
      std::is_base_of<StateReactor, TSmaccObjectType>::value ||
      std::is_base_of<ISmaccComponent, TSmaccObjectType>::value,
    "Only are accepted smacc types as subscribers for smacc signals");

  typedef decltype(callback) ft;
  Bind<boost::function_types::function_arity<ft>::value> binder;
  boost::signals2::connection connection = binder.bindaux(signal, callback, object);

  // long life-time objects
  if (
    std::is_base_of<ISmaccComponent, TSmaccObjectType>::value ||
    std::is_base_of<ISmaccClient, TSmaccObjectType>::value ||
    std::is_base_of<ISmaccOrthogonal, TSmaccObjectType>::value ||
    std::is_base_of<ISmaccStateMachine, TSmaccObjectType>::value)
  {
  }
  else if (
    std::is_base_of<ISmaccState, TSmaccObjectType>::value ||
    std::is_base_of<StateReactor, TSmaccObjectType>::value ||
    std::is_base_of<ISmaccClientBehavior, TSmaccObjectType>::value)
  {
    RCLCPP_INFO(
      getLogger(),
      "[StateMachine] life-time constrained smacc signal subscription created. Subscriber is %s",
      demangledTypeName<TSmaccObjectType>().c_str());
    stateCallbackConnections.push_back(connection);
  }
  else  // state life-time objects
  {
    RCLCPP_WARN(
      getLogger(),
      "[StateMachine] connecting signal to an unknown object with life-time unknown "
      "behavior. It might provoke "
      "an exception if the object is destroyed during the execution.");
  }

  return connection;
}

// template <typename TSmaccSignal, typename TMemberFunctionPrototype>
// boost::signals2::connection ISmaccStateMachine::createSignalConnection(TSmaccSignal &signal,
// TMemberFunctionPrototype callback)
// {
//     return signal.connect(callback);
//     // return signal;
// }

template <typename T>
bool ISmaccStateMachine::getParam(std::string param_name, T & param_storage)
{
  return getNode()->get_parameter(param_name, param_storage);
}

// Delegates to ROS param access with the current NodeHandle
template <typename T>
void ISmaccStateMachine::setParam(std::string param_name, T param_val)
{
  getNode()->set_parameter(rclcpp::Parameter(param_name, param_val));
}

// Delegates to ROS param access with the current NodeHandle
template <typename T>
bool ISmaccStateMachine::param(std::string param_name, T & param_val, const T & default_val)
{
  return getNode()->declare_parameter(param_name, param_val, default_val);
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateEntryStart(StateType * state)
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex_);

  RCLCPP_DEBUG(
    getLogger(),
    "[State Machne] Initializating a new state '%s' and updating current state. Getting state "
    "meta-information. number of orthogonals: %ld",
    demangleSymbol(typeid(StateType).name()).c_str(), this->orthogonals_.size());

  stateSeqCounter_++;
  currentState_ = state;
  currentStateInfo_ = stateMachineInfo_->getState<StateType>();
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateEntryEnd(StateType *)
{
  RCLCPP_INFO(
    getLogger(), "[%s] State OnEntry code finished",
    demangleSymbol(typeid(StateType).name()).c_str());

  for (auto pair : this->orthogonals_)
  {
    // RCLCPP_INFO(getLogger(),"ortho onentry: %s", pair.second->getName().c_str());
    auto & orthogonal = pair.second;
    try
    {
      orthogonal->onEntry();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[Orthogonal %s] Exception on Entry - continuing with next orthogonal. Exception info: %s",
        pair.second->getName().c_str(), e.what());
    }
  }

  for (auto & sr : this->currentState_->getStateReactors())
  {
    auto srname = smacc2::demangleSymbol(typeid(*sr).name());
    RCLCPP_INFO_STREAM(getLogger(), "state reactor onEntry: " << srname);
    try
    {
      sr->onEntry();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[State Reactor %s] Exception on Entry - continuing with next state reactor. Exception "
        "info: %s",
        srname.c_str(), e.what());
    }
  }

  for (auto & eg : this->currentState_->getEventGenerators())
  {
    auto egname = smacc2::demangleSymbol(typeid(*eg).name());
    RCLCPP_INFO_STREAM(getLogger(), "event generator onEntry: " << egname);
    try
    {
      eg->onEntry();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[Event generator %s] Exception on Entry - continuing with next state reactor. Exception "
        "info: %s",
        egname.c_str(), e.what());
    }
  }

  this->updateStatusMessage();
  stateMachineCurrentAction = StateMachineInternalAction::STATE_STEADY;
}

template <typename StateType>
void ISmaccStateMachine::notifyOnRuntimeConfigurationFinished(StateType *)
{
  for (auto pair : this->orthogonals_)
  {
    // RCLCPP_INFO(getLogger(),"ortho onruntime configure: %s", pair.second->getName().c_str());
    auto & orthogonal = pair.second;
    orthogonal->runtimeConfigure();
  }

  this->updateStatusMessage();

  stateMachineCurrentAction = StateMachineInternalAction::STATE_ENTERING;
}

template <typename StateType>
void ISmaccStateMachine::notifyOnRuntimeConfigured(StateType *)
{
  stateMachineCurrentAction = StateMachineInternalAction::STATE_CONFIGURING;
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateExitting(StateType * state)
{
  stateMachineCurrentAction = StateMachineInternalAction::STATE_EXITING;

  auto fullname = demangleSymbol(typeid(StateType).name());
  RCLCPP_WARN_STREAM(getLogger(), "exiting state: " << fullname);
  // this->set_parameter("destroyed", true);

  RCLCPP_INFO_STREAM(getLogger(), "Notification State Exit: leaving state" << state);
  for (auto pair : this->orthogonals_)
  {
    auto & orthogonal = pair.second;
    try
    {
      orthogonal->onExit();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[Orthogonal %s] Exception onExit - continuing with next orthogonal. Exception info: %s",
        pair.second->getName().c_str(), e.what());
    }
  }

  for (auto & sr : state->getStateReactors())
  {
    auto srname = smacc2::demangleSymbol(typeid(*sr).name());
    RCLCPP_INFO_STREAM(getLogger(), "state reactor OnExit: " << srname);
    try
    {
      sr->onExit();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[State Reactor %s] Exception on OnExit - continuing with next state reactor. Exception "
        "info: %s",
        srname.c_str(), e.what());
    }
  }

  for (auto & eg : state->getEventGenerators())
  {
    auto egname = smacc2::demangleSymbol(typeid(*eg).name());
    RCLCPP_INFO_STREAM(getLogger(), "event generator OnExit: " << egname);
    try
    {
      eg->onExit();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[State Reactor %s] Exception on OnExit - continuing with next state reactor. Exception "
        "info: %s",
        egname.c_str(), e.what());
    }
  }

  this->lockStateMachine("state exit");

  for (auto & conn : this->stateCallbackConnections)
  {
    RCLCPP_WARN_STREAM(
      getLogger(),
      "[StateMachine] Disconnecting scoped-lifetime SmaccSignal "
      "subscription");
    conn.disconnect();
  }

  this->stateCallbackConnections.clear();

  currentState_ = nullptr;
}

template <typename StateType>
void ISmaccStateMachine::notifyOnStateExited(StateType *)
{
  auto fullname = demangleSymbol(typeid(StateType).name());

  // then call exit state
  RCLCPP_WARN_STREAM(getLogger(), "state exit: " << fullname);

  stateMachineCurrentAction = StateMachineInternalAction::TRANSITIONING;
  this->unlockStateMachine("state exit");
}
//------------------------------------------------------------------------------------------------
template <typename EventType>
void ISmaccStateMachine::propagateEventToStateReactors(ISmaccState * st, EventType * ev)
{
  RCLCPP_DEBUG(
    getLogger(), "PROPAGATING EVENT [%s] TO LUs [%s]: ", demangleSymbol<EventType>().c_str(),
    st->getClassName().c_str());
  for (auto & sb : st->getStateReactors())
  {
    sb->notifyEvent(ev);
  }

  auto * pst = st->getParentState();
  if (pst != nullptr)
  {
    propagateEventToStateReactors(pst, ev);
  }
}

template <typename InitialStateType>
void ISmaccStateMachine::buildStateMachineInfo()
{
  this->stateMachineInfo_ = std::make_shared<SmaccStateMachineInfo>(this->getNode());
  this->stateMachineInfo_->buildStateMachineInfo<InitialStateType>();
  this->stateMachineInfo_->assembleSMStructureMessage(this);
  this->checkStateMachineConsistence();
}

uint64_t ISmaccStateMachine::getCurrentStateCounter() const { return this->stateSeqCounter_; }

ISmaccState * ISmaccStateMachine::getCurrentState() const { return this->currentState_; }

const smacc2::introspection::SmaccStateMachineInfo & ISmaccStateMachine::getStateMachineInfo()
{
  return *this->stateMachineInfo_;
}

}  // namespace smacc2
