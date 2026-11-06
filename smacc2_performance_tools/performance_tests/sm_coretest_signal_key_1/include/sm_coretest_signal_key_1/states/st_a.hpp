// Copyright 2025 Robosoft Inc.
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
#pragma once

namespace sm_coretest_signal_key_1
{
// StA owns the mixin-first subscriber for 1 s. When StA exits, the subscriber is disposed
// and destroyed; the tick component keeps firing at 5 Hz into StRest.
//
// StA ALSO subscribes to the same tick itself, with `this` (defect F in the study): the
// state object is destroyed by Boost.Statechart right after exit(), and nothing in the
// core erases the state's own entry from stateCallbackConnections. A "[StA] STATE TICK"
// line logged while StRest is active is a callback into a destroyed state.
struct StA : smacc2::SmaccState<StA, SmCoretestSignalKey1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StRest>> reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(1s);
    configure_orthogonal<OrTick, CbMixinFirstSubscriber>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StA] Entered. Subscriber alive for 1s.");
    // ISmaccState::requiresComponent does not compile on this tree (study finding N); reach
    // the component through the orthogonal and client instead.
    tick_ = this->getOrthogonal<OrTick>()
              ->getClients()
              .front()
              ->template getComponent<smacc2::client_core_components::CpRos2Timer>();
    if (tick_ != nullptr)
    {
      tick_->onTimerTick(&StA::onStateTick, this);
      RCLCPP_INFO(
        getLogger(), "[StA] state subscribed to onTimerTick with this=%p",
        static_cast<void *>(this));
    }
  }

  void onStateTick()
  {
    ++stateTicks_;
    RCLCPP_INFO(getLogger(), "[StA] STATE TICK %d", stateTicks_);
  }
  void onExit() { RCLCPP_INFO(getLogger(), "[StA] Exiting. state ticks=%d", stateTicks_); }

  smacc2::client_core_components::CpRos2Timer * tick_ = nullptr;
  int stateTicks_ = 0;
};
}  // namespace sm_coretest_signal_key_1
