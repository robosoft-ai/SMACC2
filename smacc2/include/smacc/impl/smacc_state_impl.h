/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/introspection/introspection.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_orthogonal.h>
#include <smacc/smacc_state.h>
#include <smacc/smacc_state_reactor.h>
//#include <smacc/smacc_event_generator.h>
#include <smacc/introspection/string_type_walker.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
using namespace smacc::introspection;
//-------------------------------------------------------------------------------------------------------
// Delegates to ROS param access with the current NodeHandle
template <typename T>
bool ISmaccState::getParam(std::string param_name, T & param_storage)
{
  RCLCPP_INFO_STREAM(
    this->getNode()->get_logger(), "getParam from node: " << getNode()->get_namespace());
  return getNode()->get_parameter(param_name, param_storage);
}
//-------------------------------------------------------------------------------------------------------

// Delegates to ROS param access with the current NodeHandle
template <typename T>
void ISmaccState::setParam(std::string param_name, T param_val)
{
  getNode()->set_parameter(rclcpp::Parameter(param_name, param_val));
}
//-------------------------------------------------------------------------------------------------------
template <typename T>
void ISmaccState::param(std::string param_name, T default_value)
{
  getNode()->declare_parameter(param_name, default_value);
}
//-------------------------------------------------------------------------------------------------------
#define THIS_STATE_NAME ((demangleSymbol(typeid(*this).name()).c_str()))
template <typename TOrthogonal, typename TBehavior, typename... Args>
std::shared_ptr<TBehavior> ISmaccState::configure(Args &&... args)
{
  std::string orthogonalkey = demangledTypeName<TOrthogonal>();
  RCLCPP_INFO(
    getNode()->get_logger(), "[%s] Configuring orthogonal: %s", THIS_STATE_NAME,
    orthogonalkey.c_str());

  TOrthogonal * orthogonal = this->getOrthogonal<TOrthogonal>();
  if (orthogonal != nullptr)
  {
    auto clientBehavior = std::shared_ptr<TBehavior>(new TBehavior(args...));
    clientBehavior->currentState = this;
    orthogonal->addClientBehavior(clientBehavior);
    clientBehavior->template onOrthogonalAllocation<TOrthogonal, TBehavior>();
    return clientBehavior;
  }
  else
  {
    RCLCPP_ERROR(
      getNode()->get_logger(),
      "[%s] Skipping client behavior creation in orthogonal [%s]. It does not exist.",
      THIS_STATE_NAME, orthogonalkey.c_str());
    return nullptr;
  }
}
//-------------------------------------------------------------------------------------------------------

template <typename SmaccComponentType>
void ISmaccState::requiresComponent(SmaccComponentType *& storage)
{
  this->getStateMachine().requiresComponent(storage);
}
//-------------------------------------------------------------------------------------------------------

template <typename SmaccClientType>
void ISmaccState::requiresClient(SmaccClientType *& storage)
{
  const char * sname = (demangleSymbol(typeid(*this).name()).c_str());
  storage = nullptr;
  auto & orthogonals = this->getStateMachine().getOrthogonals();
  for (auto & ortho : orthogonals)
  {
    ortho.second->requiresClient(storage);
    if (storage != nullptr) return;
  }

  RCLCPP_ERROR(
    getNode()->get_logger(),
    "[%s] Client of type '%s' not found in any orthogonal of the current state machine. This may "
    "produce a segmentation fault if the returned reference is used.",
    sname, demangleSymbol<SmaccClientType>().c_str());
}
//-------------------------------------------------------------------------------------------------------

template <typename T>
bool ISmaccState::getGlobalSMData(std::string name, T & ret)
{
  return this->getStateMachine().getGlobalSMData(name, ret);
}
//-------------------------------------------------------------------------------------------------------

// Store globally in this state machine. (By value parameter )
template <typename T>
void ISmaccState::setGlobalSMData(std::string name, T value)
{
  this->getStateMachine().setGlobalSMData(name, value);
}
//-------------------------------------------------------------------------------------------------------

template <typename TStateReactor, typename... TEvArgs>
std::shared_ptr<TStateReactor> ISmaccState::createStateReactor(TEvArgs... args)
{
  auto sr = std::make_shared<TStateReactor>(args...);
  //sb->initialize(this, mock);
  //sb->setOutputEvent(typelist<TTriggerEvent>());
  stateReactors_.push_back(sr);
  return sr;
}

template <typename TEventGenerator, typename... TEvArgs>
std::shared_ptr<TEventGenerator> ISmaccState::createEventGenerator(TEvArgs... args)
{
  auto eg = std::make_shared<TEventGenerator>(args...);
  eventGenerators_.push_back(eg);
  return eg;
}

// used to iterate on the source events list and fill the information of the stateReactorInfo structure
template <typename TEventList>
struct AddTEventTypeStateReactorInfo
{
  smacc::SmaccStateReactorInfo * srInfo_;
  AddTEventTypeStateReactorInfo(smacc::SmaccStateReactorInfo * srInfo) : srInfo_(srInfo) {}

  template <typename T>
  void operator()(T)
  {
    auto evinfo = std::make_shared<SmaccEventInfo>(TypeInfo::getTypeInfoFromType<T>());
    srInfo_->sourceEventTypes.push_back(evinfo);
    EventLabel<T>(evinfo->label);
  }
};

// used to iterate on the source events list and fill the information of the stateReactorInfo structure
// (is it required alreadyy having the AddTEventTypeStateReactorInfo?)
template <typename TEventList>
struct AddTEventTypeStateReactor
{
  smacc::StateReactor * sr_;
  AddTEventTypeStateReactor(smacc::StateReactor * sr) : sr_(sr) {}

  template <typename T>
  void operator()(T)
  {
    sr_->addInputEvent<T>();
  }
};

template <typename TStateReactor, typename TTriggerEvent, typename TEventList, typename... TEvArgs>
std::shared_ptr<TStateReactor> ISmaccState::createStateReactor(TEvArgs... args)
{
  auto sr = std::make_shared<TStateReactor>(args...);
  sr->initialize(this);
  sr->template setOutputEvent<TTriggerEvent>();

  using boost::mpl::_1;
  using wrappedList = typename boost::mpl::transform<TEventList, _1>::type;
  AddTEventTypeStateReactor<TEventList> op(sr.get());
  boost::mpl::for_each<wrappedList>(op);

  stateReactors_.push_back(sr);
  return sr;
}

template <typename TOrthogonal>
TOrthogonal * ISmaccState::getOrthogonal()
{
  return this->getStateMachine().getOrthogonal<TOrthogonal>();
}

template <typename TEventGenerator>
TEventGenerator * ISmaccState::getEventGenerator()
{
  TEventGenerator * ret = nullptr;
  for (auto & evg : this->eventGenerators_)
  {
    ret = dynamic_cast<TEventGenerator *>(evg.get());
    if (ret != nullptr) break;
  }
  return ret;
}

template <typename TStateReactor>
TStateReactor * ISmaccState::getStateReactor()
{
  TStateReactor * ret = nullptr;
  for (auto & sr : this->eventGenerators_)
  {
    ret = dynamic_cast<TStateReactor *>(sr.get());
    if (ret != nullptr) break;
  }
  return ret;
}

// template <typename TStateReactor, typename TTriggerEvent, typename... TEvArgs>
// std::shared_ptr<TStateReactor> ISmaccState::createStateReactor(TEvArgs... args)
// {
//     auto sb = std::make_shared<TStateReactor>(std::forward(args...));
//     sb->initialize(this, typelist<TEvArgs...>());
//     sb->setOutputEvent(typelist<TTriggerEvent>());
//     stateReactors_.push_back(sb);

//     return sb;
// }
//-------------------------------------------------------------------------------------------------------

template <typename EventType>
void ISmaccState::postEvent(const EventType & ev)
{
  getStateMachine().postEvent(ev);
}

template <typename EventType>
void ISmaccState::postEvent()
{
  getStateMachine().postEvent<EventType>();
}
//-------------------------------------------------------------------------------------------------------

template <typename TransitionType>
void ISmaccState::notifyTransition()
{
  auto transitionType = TypeInfo::getTypeInfoFromType<TransitionType>();
  this->notifyTransitionFromTransitionTypeInfo(transitionType);
}

//-------------------------------------------------------------------------------------------------------------------

}  // namespace smacc

// implementation depends on state definition
#include <smacc/impl/smacc_event_generator_impl.h>
#include <smacc/impl/smacc_state_reactor_impl.h>
