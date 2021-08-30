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
void SmaccAsyncClientBehavior::onOrthogonalAllocation()
{
  postFinishEventFn_ = [=] {
    this->onFinished_();
    this->postEvent<EvCbFinished<TSourceObject, TOrthogonal>>();
  };

  postSuccessEventFn_ = [=] {
    this->onSuccess_();
    this->postEvent<EvCbSuccess<TSourceObject, TOrthogonal>>();
  };

  postFailureEventFn_ = [=] {
    this->onFailure_();
    this->postEvent<EvCbFailure<TSourceObject, TOrthogonal>>();
  };
}

template <typename TCallback, typename T>
boost::signals2::connection SmaccAsyncClientBehavior::onSuccess(TCallback callback, T * object)
{
  return this->getStateMachine()->createSignalConnection(onSuccess_, callback, object);
}

template <typename TCallback, typename T>
boost::signals2::connection SmaccAsyncClientBehavior::onFinished(TCallback callback, T * object)
{
  return this->getStateMachine()->createSignalConnection(onFinished_, callback, object);
}

template <typename TCallback, typename T>
boost::signals2::connection SmaccAsyncClientBehavior::onFailure(TCallback callback, T * object)
{
  return this->getStateMachine()->createSignalConnection(onFailure_, callback, object);
}
}  // namespace smacc2
