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

#include <memory>

#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_reactor.hpp>
#include "rclcpp/rclcpp.hpp"

namespace smacc2
{
StateReactor::StateReactor() {}

StateReactor::~StateReactor() {}

void StateReactor::initialize(smacc2::ISmaccState * ownerState)
{
  this->ownerState = ownerState;
  this->onInitialized();
}

void StateReactor::onInitialized() {}

void StateReactor::onEventNotified(const std::type_info * /*eventType*/) {}

void StateReactor::onEntry() {}

void StateReactor::onExit() {}

void StateReactor::update()
{
  if (this->triggers())
  {
    RCLCPP_INFO(getLogger(), "State reactor base REALLY TRIGGERS!!");
    this->postEventFn();
  }
}

rclcpp::Node::SharedPtr StateReactor::getNode() { return ownerState->getNode(); }

namespace introspection
{
void StateReactorHandler::configureStateReactor(std::shared_ptr<smacc2::StateReactor> sb)
{
  for (auto callback : this->callbacks_)
  {
    callback.fn(sb);
  }
}

void EventGeneratorHandler::configureEventGenerator(std::shared_ptr<smacc2::SmaccEventGenerator> eg)
{
  for (auto callback : this->callbacks_)
  {
    callback.fn(eg);
  }
}
}  // namespace introspection
}  // namespace smacc2
