// Copyright 2026 RobosoftAI Inc.
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

namespace sm_coretest_object_churn_1
{
// One of the two churn states; see the state machine header for what each state holds.
struct StChurnB : smacc2::SmaccState<StChurnB, SmCoretestObjectChurn1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvAllGo<SrAllEventsGo, StChurnB>, StChurnA>> reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTick, CbTickSubscriber>();
    configure_orthogonal<OrAsync, CbFinish>();
    static_createEventGenerator<EgConditionalGenerator>(ConditionalGeneratorMode::ON_UPDATE);
    static_createStateReactor<
      SrAllEventsGo, EvAllGo<SrAllEventsGo, StChurnB>,
      mpl::list<EvCbSuccess<CbFinish, OrAsync>, EvTrue<EgConditionalGenerator, StChurnB>>>();
  }

  void runtimeConfigure()
  {
    this->getEventGenerator<EgConditionalGenerator>()->setPredicateFunction([] { return true; });
  }

  void onEntry()
  {
    this->requiresComponent(tick_);
    tick_->onTimerTick(&StChurnB::onTick, this);
  }

  void onTick() { ++ticks_; }

  smacc2::client_core_components::CpRos2Timer * tick_ = nullptr;
  unsigned long ticks_ = 0;
};
}  // namespace sm_coretest_object_churn_1
