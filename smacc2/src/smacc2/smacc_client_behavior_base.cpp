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

#include <smacc2/smacc_client_behavior.hpp>

namespace smacc2
{
ISmaccClientBehavior::ISmaccClientBehavior()
{
  stateMachine_ = nullptr;
  currentState = nullptr;
}

ISmaccClientBehavior::~ISmaccClientBehavior()
{
  RCLCPP_WARN(getLogger(), "Client behavior deallocated.");
}

std::string ISmaccClientBehavior::getName() const { return demangleSymbol(typeid(*this).name()); }

rclcpp::Node::SharedPtr ISmaccClientBehavior::getNode() { return this->stateMachine_->getNode(); }

rclcpp::Logger ISmaccClientBehavior::getLogger()
{
  auto nh = this->getNode();
  if (nh != nullptr)
  {
    return nh->get_logger();
  }
  else
  {
    return rclcpp::get_logger("SMACC");
  }
}

void ISmaccClientBehavior::runtimeConfigure()
{
  RCLCPP_DEBUG(
    getLogger(), "[%s] Default empty SmaccClientBehavior runtimeConfigure",
    this->getName().c_str());
}

void ISmaccClientBehavior::executeOnEntry()
{
  RCLCPP_DEBUG(
    getLogger(), "[%s] Default empty SmaccClientBehavior onEntry", this->getName().c_str());
  this->onEntry();
}

void ISmaccClientBehavior::executeOnExit()
{
  RCLCPP_DEBUG(
    getLogger(), "[%s] Default empty SmaccClientBehavior onExit", this->getName().c_str());
  this->onExit();
}

void ISmaccClientBehavior::dispose() {}

}  // namespace smacc2
