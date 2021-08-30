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

template <typename TComponent>
void ISmaccComponent::requiresComponent(TComponent *& requiredComponentStorage)
{
  requiredComponentStorage = this->owner_->getComponent<TComponent>();
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
