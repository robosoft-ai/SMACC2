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

#include <smacc2/impl/smacc_client_impl.hpp>
#include <smacc2/smacc_client.hpp>

namespace smacc2
{
ISmaccClient::ISmaccClient() {}

ISmaccClient::~ISmaccClient() {}

void ISmaccClient::initialize() { this->onInitialize(); }

void ISmaccClient::onInitialize() {}

void ISmaccClient::getComponents(std::vector<std::shared_ptr<ISmaccComponent>> & components)
{
  for (auto & ce : components_)
  {
    components.push_back(ce.second);
  }
}

std::string ISmaccClient::getName() const
{
  std::string keyname = demangleSymbol(typeid(*this).name());
  return keyname;
}

void ISmaccClient::setStateMachine(ISmaccStateMachine * stateMachine)
{
  stateMachine_ = stateMachine;
}

void ISmaccClient::setOrthogonal(ISmaccOrthogonal * orthogonal) { orthogonal_ = orthogonal; }

smacc2::introspection::TypeInfo::Ptr ISmaccClient::getType()
{
  return smacc2::introspection::TypeInfo::getFromStdTypeInfo(typeid(*this));
}

rclcpp::Node::SharedPtr ISmaccClient::getNode() { return this->getStateMachine()->getNode(); }

}  // namespace smacc2
