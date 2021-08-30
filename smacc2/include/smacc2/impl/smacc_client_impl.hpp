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

#include <smacc2/impl/smacc_state_machine_impl.hpp>
#include <smacc2/smacc_client.hpp>

namespace smacc2
{
template <typename EventType>
void ISmaccClient::postEvent(const EventType & ev)
{
  stateMachine_->postEvent(ev);
}

template <typename EventType>
void ISmaccClient::postEvent()
{
  stateMachine_->postEvent<EventType>();
}

template <typename TComponent>
TComponent * ISmaccClient::getComponent()
{
  return this->getComponent<TComponent>(std::string());
}

template <typename TComponent>
TComponent * ISmaccClient::getComponent(std::string name)
{
  for (auto & component : components_)
  {
    if (component.first.name != name) continue;

    auto * tcomponent = dynamic_cast<TComponent *>(component.second.get());
    if (tcomponent != nullptr)
    {
      return tcomponent;
    }
  }

  return nullptr;
}

//inline
ISmaccStateMachine * ISmaccClient::getStateMachine() { return this->stateMachine_; }

template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
SmaccComponentType * ISmaccClient::createNamedComponent(std::string name, TArgs... targs)
{
  ComponentKey componentkey(&typeid(SmaccComponentType), name);

  std::shared_ptr<SmaccComponentType> ret;

  auto it = this->components_.find(componentkey);

  if (it == this->components_.end())
  {
    auto tname = demangledTypeName<SmaccComponentType>();
    RCLCPP_INFO(
      getLogger(),
      "Creating a new component of type %s smacc component is required. Creating a new instance %s",
      demangledTypeName<SmaccComponentType>().c_str(), tname.c_str());

    ret = std::shared_ptr<SmaccComponentType>(new SmaccComponentType(targs...));
    ret->setStateMachine(this->getStateMachine());
    ret->owner_ = this;
    ret->initialize(this);

    this->components_[componentkey] =
      ret;  //std::dynamic_pointer_cast<smacc2::ISmaccComponent>(ret);
    RCLCPP_DEBUG(getLogger(), "%s resource is required. Done.", tname.c_str());
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "%s resource is required. Found resource in cache.",
      demangledTypeName<SmaccComponentType>().c_str());
    ret = dynamic_pointer_cast<SmaccComponentType>(it->second);
  }

  ret->template onOrthogonalAllocation<TOrthogonal, TClient>();

  return ret.get();
}

template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
SmaccComponentType * ISmaccClient::createComponent(TArgs... targs)
{
  return this->createNamedComponent<SmaccComponentType, TOrthogonal, TClient>(
    std::string(), targs...);
}

template <typename TSmaccSignal, typename T>
void ISmaccClient::connectSignal(TSmaccSignal & signal, void (T::*callback)(), T * object)
{
  return this->getStateMachine()->createSignalConnection(signal, callback, object);
}

template <typename SmaccClientType>
void ISmaccClient::requiresClient(SmaccClientType *& storage)
{
  this->orthogonal_->requiresClient(storage);
}

}  // namespace smacc2
