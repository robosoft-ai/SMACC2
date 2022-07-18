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

#include <string>

#include <smacc2/component.hpp>
#include <smacc2/impl/smacc_state_machine_impl.hpp>

namespace smacc2
{
template <typename EventType>
void ISmaccComponent::postEvent(const EventType & ev)
{
  stateMachine_->postEvent(ev);
}

template <typename EventType>
void ISmaccComponent::postEvent()
{
  auto ev = new EventType();
  stateMachine_->postEvent(ev);
}

template <typename TComponent>
void ISmaccComponent::requiresComponent(
  TComponent *& requiredComponentStorage, bool throwExceptionIfNotExist)
{
  requiredComponentStorage = this->owner_->getComponent<TComponent>();

  if (requiredComponentStorage == nullptr && throwExceptionIfNotExist)
  {
    RCLCPP_DEBUG_STREAM(
      this->getLogger(), std::string("Required component ") +
                           demangleSymbol(typeid(TComponent).name()) +
                           " not found. Available components:");

    std::vector<std::shared_ptr<ISmaccComponent>> components;
    this->owner_->getComponents(components);

    for (auto c : components)
    {
      RCLCPP_DEBUG(this->getLogger(), "- Component %s", c->getName().c_str());
    }

    throw std::runtime_error(
      std::string("Component ") + demangleSymbol(typeid(TComponent).name()) + " not found");
  }
}

template <typename TComponent>
void ISmaccComponent::requiresComponent(
  std::string name, TComponent *& requiredComponentStorage, bool throwExceptionIfNotExist)
{
  requiredComponentStorage = this->owner_->getComponent<TComponent>(name);

  if (requiredComponentStorage == nullptr && throwExceptionIfNotExist)
  {
    RCLCPP_DEBUG_STREAM(
      this->getLogger(), std::string("Required component with name: '") + name + "'" +
                           demangleSymbol(typeid(TComponent).name()) +
                           " not found. Available components:");

    std::vector<std::shared_ptr<ISmaccComponent>> components;
    this->owner_->getComponents(components);

    for (auto c : components)
    {
      RCLCPP_DEBUG(this->getLogger(), " - Component %s", c->getName().c_str());
    }

    throw std::runtime_error(
      std::string("Component ") + demangleSymbol(typeid(TComponent).name()) +
      std::string(" not found"));
  }
}

template <typename TClient>
void ISmaccComponent::requiresClient(TClient *& requiredClientStorage)
{
  this->owner_->requiresClient(requiredClientStorage);
}

template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
SmaccComponentType * ISmaccComponent::createSiblingComponent(TArgs... targs)
{
  return this->owner_->createComponent<SmaccComponentType, TOrthogonal, TClient>(targs...);
}

template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
SmaccComponentType * ISmaccComponent::createSiblingNamedComponent(std::string name, TArgs... targs)
{
  return this->owner_->createNamedComponent<SmaccComponentType, TOrthogonal, TClient>(
    name, targs...);
}

}  // namespace smacc2
