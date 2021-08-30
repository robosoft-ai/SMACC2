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
#include <smacc2/component.hpp>
#include <smacc2/impl/smacc_component_impl.hpp>
namespace smacc2
{
ISmaccComponent::~ISmaccComponent() {}

ISmaccComponent::ISmaccComponent() : owner_(nullptr) {}

void ISmaccComponent::initialize(ISmaccClient * owner)
{
  owner_ = owner;
  this->onInitialize();
}

void ISmaccComponent::onInitialize() {}

void ISmaccComponent::setStateMachine(ISmaccStateMachine * stateMachine)
{
  stateMachine_ = stateMachine;
}

rclcpp::Node::SharedPtr ISmaccComponent::getNode() { return this->owner_->getNode(); }

rclcpp::Logger ISmaccComponent::getLogger() { return getLogger(); }

std::string ISmaccComponent::getName() const
{
  std::string keyname = demangleSymbol(typeid(*this).name());
  return keyname;
}
}  // namespace smacc2
