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
#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc_state_reactor.hpp>

namespace smacc2
{
template <typename EventType>
void StateReactor::postEvent(const EventType & ev)
{
  ownerState->postEvent(ev);
}

template <typename EventType>
void StateReactor::postEvent()
{
  ownerState->postEvent<EventType>();
}

template <typename TEv>
void StateReactor::setOutputEvent()
{
  this->postEventFn = [this]() {
    RCLCPP_INFO_STREAM(
      this->getLogger(), "[State Reactor Base] postingfn posting event: " << demangleSymbol<TEv>());
    auto * ev = new TEv();
    this->ownerState->getStateMachine().postEvent(ev);
  };
}

template <typename TEv>
void StateReactor::addInputEvent()
{
  this->eventTypes.push_back(&typeid(TEv));
}

template <typename T, typename TClass>
void StateReactor::createEventCallback(void (TClass::*callback)(T *), TClass * object)
{
  const auto * eventtype = &typeid(T);
  this->eventCallbacks_[eventtype] = [=](void * msg) {
    T * evptr = (T *)msg;
    (object->*callback)(evptr);
  };
}

template <typename T>
void StateReactor::createEventCallback(std::function<void(T *)> callback)
{
  const auto * eventtype = &typeid(T);
  this->eventCallbacks_[eventtype] = [=](void * msg) {
    T * evptr = (T *)msg;
    callback(evptr);
  };
}

namespace introspection
{
template <typename TEv>
void StateReactorHandler::addInputEvent()
{
  StateReactorCallbackFunctor functor;
  functor.fn = [this](std::shared_ptr<smacc2::StateReactor> sr) {
    RCLCPP_INFO(
      nh_->get_logger(), "[%s] State Reactor adding input event: %s",
      srInfo_->stateReactorType->getFullName().c_str(), demangledTypeName<TEv>().c_str());
    sr->addInputEvent<TEv>();
  };

  this->callbacks_.push_back(functor);

  auto evtype = TypeInfo::getFromStdTypeInfo(typeid(TEv));
  auto evinfo = std::make_shared<SmaccEventInfo>(evtype);
  EventLabel<TEv>(evinfo->label);

  srInfo_->sourceEventTypes.push_back(evinfo);
}

template <typename TEv>
void StateReactorHandler::setOutputEvent()
{
  StateReactorCallbackFunctor functor;
  functor.fn = [this](std::shared_ptr<smacc2::StateReactor> sr) {
    RCLCPP_INFO(
      nh_->get_logger(), "[%s] State Reactor setting output event: %s",
      srInfo_->stateReactorType->getFullName().c_str(), demangledTypeName<TEv>().c_str());
    sr->setOutputEvent<TEv>();
  };

  this->callbacks_.push_back(functor);
}
}  // namespace introspection

}  // namespace smacc2
