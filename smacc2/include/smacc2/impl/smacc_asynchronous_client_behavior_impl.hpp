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
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <smacc2/smacc_state_machine.hpp>

namespace smacc2
{
template <typename TOrthogonal, typename TSourceObject>
void SmaccAsyncClientBehavior::onStateOrthogonalAllocation()
{
  this->onOrthogonalAllocation<TOrthogonal, TSourceObject>();
}

template <typename TOrthogonal, typename TSourceObject>
void SmaccAsyncClientBehavior::onOrthogonalAllocation()
{
  if (postFinishEventFn_ || postSuccessEventFn_ || postFailureEventFn_)
  {
    RCLCPP_WARN(
      getLogger(),
      "SmaccAsyncClientBehavior already has event posting functions assigned. Skipping "
      "re-assignment. This could be a problem if you are using the same behavior in multiple "
      "states. This may be related with the deprecation of onOrthogonalAllocation in favor of "
      "onStateOrthogonalAllocation.");

    return;
  }

  postFinishEventFn_ = [this]
  {
    this->onFinished_();
    this->postEvent<EvCbFinished<TSourceObject, TOrthogonal>>();
  };

  postSuccessEventFn_ = [this]
  {
    this->onSuccess_();
    this->postEvent<EvCbSuccess<TSourceObject, TOrthogonal>>();
  };

  postFailureEventFn_ = [this]
  {
    this->onFailure_();
    this->postEvent<EvCbFailure<TSourceObject, TOrthogonal>>();
  };
}

template <typename TCallbackMethod, typename T>
boost::signals2::connection SmaccAsyncClientBehavior::onSuccess(
  TCallbackMethod callback, T * object)
{
  return this->getStateMachine()->createSignalConnection(onSuccess_, callback, object);
}

template <typename TCallbackMethod, typename T>
boost::signals2::connection SmaccAsyncClientBehavior::onFinished(
  TCallbackMethod callback, T * object)
{
  return this->getStateMachine()->createSignalConnection(onFinished_, callback, object);
}

template <typename TCallbackMethod, typename T>
boost::signals2::connection SmaccAsyncClientBehavior::onFailure(
  TCallbackMethod callback, T * object)
{
  return this->getStateMachine()->createSignalConnection(onFailure_, callback, object);
}
}  // namespace smacc2
