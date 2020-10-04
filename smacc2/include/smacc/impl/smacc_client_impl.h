/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <smacc/impl/smacc_state_machine_impl.h>

namespace smacc
{
    template <typename EventType>
    void ISmaccClient::postEvent(const EventType &ev)
    {
        stateMachine_->postEvent(ev);
    }

    template <typename EventType>
    void ISmaccClient::postEvent()
    {
        stateMachine_->postEvent<EventType>();
    }

    template <typename TComponent>
    TComponent *ISmaccClient::getComponent()
    {
        return this->getComponent<TComponent>(std::string());
    }

    template <typename TComponent>
    TComponent *ISmaccClient::getComponent(std::string name)
    {
        for (auto &component : components_)
        {
            if (component.first.name != name)
                continue;

            auto *tcomponent = dynamic_cast<TComponent *>(component.second.get());
            if (tcomponent != nullptr)
            {
                return tcomponent;
            }
        }

        return nullptr;
    }

    //inline
    ISmaccStateMachine *ISmaccClient::getStateMachine()
    {
        return this->stateMachine_;
    }

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *ISmaccClient::createNamedComponent(std::string name, TArgs... targs)
    {
        ComponentKey componentkey(&typeid(SmaccComponentType), name);

        std::shared_ptr<SmaccComponentType> ret;

        auto it = this->components_.find(componentkey);

        if (it == this->components_.end())
        {
            auto tname = demangledTypeName<SmaccComponentType>();
            RCLCPP_DEBUG(getNode()->get_logger(),"%s smacc component is required. Creating a new instance.", tname.c_str());

            ret = std::shared_ptr<SmaccComponentType>(new SmaccComponentType(targs...));
            ret->setStateMachine(this->getStateMachine());
            ret->owner_ = this;
            ret->initialize(this);

            this->components_[componentkey] = ret; //std::dynamic_pointer_cast<smacc::ISmaccComponent>(ret);
            RCLCPP_DEBUG(getNode()->get_logger(),"%s resource is required. Done.", tname.c_str());
        }
        else
        {
            RCLCPP_DEBUG(getNode()->get_logger(),"%s resource is required. Found resource in cache.", demangledTypeName<SmaccComponentType>().c_str());
            ret = dynamic_pointer_cast<SmaccComponentType>(it->second);
        }

        ret->template onOrthogonalAllocation<TOrthogonal, TClient>();

        return ret.get();
    }

    template <typename SmaccComponentType, typename TOrthogonal, typename TClient, typename... TArgs>
    SmaccComponentType *ISmaccClient::createComponent(TArgs... targs)
    {
        return this->createNamedComponent<SmaccComponentType, TOrthogonal, TClient>(std::string(), targs...);
    }

    template <typename TSmaccSignal, typename T>
    void ISmaccClient::connectSignal(TSmaccSignal &signal, void (T::*callback)(), T *object)
    {
        return this->getStateMachine()->createSignalConnection(signal, callback, object);
    }

    template <typename SmaccClientType>
    void ISmaccClient::requiresClient(SmaccClientType *&storage)
    {
        this->orthogonal_->requiresClient(storage);
    }

} // namespace smacc