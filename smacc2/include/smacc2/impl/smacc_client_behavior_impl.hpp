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

#include <smacc2/smacc_client_behavior.hpp>
#include <smacc2/smacc_state_machine.hpp>

namespace smacc2
{
template <typename EventType>
void ISmaccClientBehavior::postEvent(const EventType & ev)
{
  if (stateMachine_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(),
      "The client behavior cannot post events before being assigned to an orthogonal. Ignoring "
      "post event call.");
  }
  else
  {
    stateMachine_->postEvent(ev, EventLifeTime::CURRENT_STATE);
  }
}

template <typename EventType>
void ISmaccClientBehavior::postEvent()
{
  if (stateMachine_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(),
      "The client behavior cannot post events before being assigned to an orthogonal. Ignoring "
      "post event call.");
  }
  else
  {
    stateMachine_->template postEvent<EventType>(EventLifeTime::CURRENT_STATE);
  }
}

//inline
ISmaccStateMachine * ISmaccClientBehavior::getStateMachine() { return this->stateMachine_; }

//inline
ISmaccState * ISmaccClientBehavior::getCurrentState() { return this->currentState; }

template <typename SmaccClientType>
void ISmaccClientBehavior::requiresClient(SmaccClientType *& storage)
{
  currentOrthogonal->requiresClient(storage);
}

template <typename SmaccComponentType>
void ISmaccClientBehavior::requiresComponent(SmaccComponentType *& storage)
{
  if (stateMachine_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(),
      "Cannot use the requiresComponent funcionality before assigning the client behavior to an "
      "orthogonal. Try using the OnEntry method to capture required components.");
  }
  else
  {
    stateMachine_->requiresComponent(storage);
  }
}

template <typename TOrthogonal, typename TSourceObject>
void ISmaccClientBehavior::onOrthogonalAllocation()
{
}

}  // namespace smacc2
