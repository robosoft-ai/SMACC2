/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/component.h>
#include <smacc/impl/smacc_state_machine_impl.h>

namespace smacc
{
    template <typename EventType>
    void ISmaccComponent::postEvent(const EventType &ev)
    {
        stateMachine_->postEvent(ev);
    }

    template <typename TComponent>
    void ISmaccComponent::requiresComponent(TComponent *&requiredComponentStorage)
    {
        requiredComponentStorage = this->owner_->getComponent<TComponent>();
    }

    template <typename TClient>
    void ISmaccComponent::requiresClient(TClient *&requiredClientStorage)
    {
        this->owner_->requiresClient(requiredClientStorage);
    }

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *ISmaccComponent::createSiblingComponent(TArgs... targs)
    {
        return this->owner_->createComponent<SmaccComponentType, TOrthogonal, TClient>(targs...);
    }

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *ISmaccComponent::createSiblingNamedComponent(std::string name, TArgs... targs)
    {
        return this->owner_->createNamedComponent<SmaccComponentType, TOrthogonal, TClient>(name, targs...);
    }

} // namespace smacc