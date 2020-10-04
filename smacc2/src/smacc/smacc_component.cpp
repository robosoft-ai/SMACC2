/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc/component.h>
#include <smacc/impl/smacc_component_impl.h>
namespace smacc
{

ISmaccComponent::~ISmaccComponent()
{
}

ISmaccComponent::ISmaccComponent()
    : owner_(nullptr)
{
}

void ISmaccComponent::initialize(ISmaccClient *owner)
{
    owner_ = owner;
    this->onInitialize();
}

void ISmaccComponent::onInitialize()
{

}

void ISmaccComponent::setStateMachine(ISmaccStateMachine *stateMachine)
{
    stateMachine_ = stateMachine;
}

std::string ISmaccComponent::getName() const
{
    std::string keyname = demangleSymbol(typeid(*this).name());
    return keyname;
}
} // namespace smacc
