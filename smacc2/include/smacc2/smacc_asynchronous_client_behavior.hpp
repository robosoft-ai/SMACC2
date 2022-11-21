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
#include <future>
#include <smacc2/smacc_client_behavior_base.hpp>
#include <smacc2/smacc_signal.hpp>

namespace smacc2
{
template <typename AsyncCB, typename Orthogonal>
struct EvCbFinished : sc::event<EvCbFinished<AsyncCB, Orthogonal>>
{
};

template <typename AsyncCB, typename Orthogonal>
struct EvCbSuccess : sc::event<EvCbSuccess<AsyncCB, Orthogonal>>
{
};

template <typename AsyncCB, typename Orthogonal>
struct EvCbFailure : sc::event<EvCbFailure<AsyncCB, Orthogonal>>
{
};

// INTRODUCTION: Conceptually, AsynchronousClientBehaviors start in parallel on state entry.
// Asnchronous client behaviors are used when the onEntry or onExit function execution could be too much slow and
// could block the state machine thread.
// AsynchronousClientBehaviors are related with the concept orthogonality of Smacc State Machines.
// ASYNCHRONOUS STATE MACHINES DESIGN NOTES: Asynchronous behaviors can safely post events and use its local methods,
//  but the interaction with other components or elements of
// the state machine is not by-default thread safe and must be manually implemented. For example, if some element of the architecture
// (components, states, clients) need to access to this behavior client information it is needed to implement a mutex for the internal
// state of this behavior. Other example: if this behavior access to some component located in other thread, it is also may be needed
// to some mutex for that component
// ALTERNATIVE: for long duration behaviors: using default-synchromous SmaccClientBehaviors with the update method

class SmaccAsyncClientBehavior : public ISmaccClientBehavior
{
public:
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation();

  virtual ~SmaccAsyncClientBehavior();

  template <typename TCallback, typename T>
  boost::signals2::connection onSuccess(TCallback callback, T * object);

  template <typename TCallback, typename T>
  boost::signals2::connection onFinished(TCallback callback, T * object);

  template <typename TCallback, typename T>
  boost::signals2::connection onFailure(TCallback callback, T * object);

  void requestForceFinish();

  // executes onExit in a new thread
  void executeOnEntry() override;

  // executes onExit in a new thread, waits first onEntry thread if it is still running
  void executeOnExit() override;
  void waitOnEntryThread(bool requestFinish);

protected:
  void postSuccessEvent();
  void postFailureEvent();

  virtual void dispose() override;

  /// \brief onEntry is executed in a new thread. However the current state cannot be left
  /// until the onEntry thread finishes. This flag can be checked from the onEntry thread to force finishing the thread.
  // All asyncrhonous client behaviors should implement the ability of interrupting the onEntry thread.
  // to avoid blocking the state machine.
  inline bool isShutdownRequested() { return isShutdownRequested_; }

private:
  void waitFutureIfNotFinished(std::optional<std::future<int>> & threadfut, bool requestFinish);

  std::optional<std::future<int>> onEntryThread_;
  std::optional<std::future<int>> onExitThread_;

  std::function<void()> postFinishEventFn_;
  std::function<void()> postSuccessEventFn_;
  std::function<void()> postFailureEventFn_;

  SmaccSignal<void()> onFinished_;
  SmaccSignal<void()> onSuccess_;
  SmaccSignal<void()> onFailure_;

  bool isShutdownRequested_ = false;
};
}  // namespace smacc2

#include <smacc2/impl/smacc_asynchronous_client_behavior_impl.hpp>
